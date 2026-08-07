import { type Vec3, vec3 } from 'mathcat';
import type { Heightfield } from 'navcat';
import type { EdgeSampler, JumpLinkBuilderConfig, Trajectory } from './types';

function overlapRange(amin: number, amax: number, bmin: number, bmax: number): boolean {
    return !(amin > bmax || amax < bmin);
}

function overlapSpan(hf: Heightfield, cx: number, cz: number, yMin: number, yMax: number): boolean {
    if (cx < 0 || cz < 0 || cx >= hf.width || cz >= hf.height) return false;

    const origY = hf.bounds[1];
    const ch = hf.cellHeight;

    for (let s = hf.spans[cx + cz * hf.width]; s; s = s.next) {
        if (overlapRange(yMin, yMax, origY + s.min * ch, origY + s.max * ch)) return true;
    }

    return false;
}

/**
 * Per-axis DDA setup in grid-cell units: the parametric distance to the first grid line
 * ({@link tMax}), the parametric step between grid lines ({@link tDelta}), and the cell
 * step direction. A constant component returns `Infinity` for both `tMax`/`tDelta`, so the
 * axis never triggers a step (see {@link segmentHitsHeightfield}).
 */
function ddaAxis(g0: number, g1: number): { tMax: number; tDelta: number; step: number } {
    const d = g1 - g0;
    if (Math.abs(d) < 1e-9) return { tMax: Infinity, tDelta: Infinity, step: 1 };
    const step = d > 0 ? 1 : -1;
    const bnd = d > 0 ? Math.floor(g0) + 1 : g0 === Math.floor(g0) ? g0 - 1 : Math.ceil(g0) - 1;
    return { tMax: (bnd - g0) / d, tDelta: Math.abs(1 / d), step };
}

/**
 * Grid DDA of a single straight chord {@link p0}→{@link p1} across one heightfield. Each
 * visited cell gets the chord's linear Y band over its `[tEnter, tExit]` sub-interval,
 * inflated by `groundTolerance` at the bottom and `agentHeight` at the top, then tested
 * against the cell's span column.
 *
 * Test-then-step ordering: the containing cell is tested before any axis steps, so a chord
 * that only drops (both axes constant — e.g. the climb's vertical second half) still tests
 * its single cell against the full Y range before exiting.
 *
 * @see http://www.cse.yorku.ca/~amana/research/grid.pdf
 */
function segmentHitsHeightfield(cfg: JumpLinkBuilderConfig, hf: Heightfield, p0: Vec3, p1: Vec3): boolean {
    const origX = hf.bounds[0];
    const origZ = hf.bounds[2];
    const cs = hf.cellSize;
    const dy = p1[1] - p0[1];

    const gx0 = (p0[0] - origX) / cs;
    const gz0 = (p0[2] - origZ) / cs;
    const axX = ddaAxis(gx0, (p1[0] - origX) / cs);
    const axZ = ddaAxis(gz0, (p1[2] - origZ) / cs);

    let cx = Math.floor(gx0);
    let cz = Math.floor(gz0);
    let tEnter = 0;

    while (true) {
        const tExit = Math.min(axX.tMax, axZ.tMax, 1.0);
        const y0 = p0[1] + tEnter * dy;
        const y1 = p0[1] + tExit * dy;
        const yMin = Math.min(y0, y1) + cfg.groundTolerance;
        const yMax = Math.max(y0, y1) + cfg.agentHeight;

        if (overlapSpan(hf, cx, cz, yMin, yMax)) return true;
        if (tExit >= 1.0) return false;

        // step to the next cell along whichever axis exits first
        if (axX.tMax < axZ.tMax) {
            tEnter = axX.tMax;
            axX.tMax += axX.tDelta;
            cx += axX.step;
        } else {
            tEnter = axZ.tMax;
            axZ.tMax += axZ.tDelta;
            cz += axZ.step;
        }
    }
}

const spine: Vec3[] = [];

/**
 * Sample the trajectory into a polyline of `tra.num_spine` points, then DDA each chord
 * across each heightfield. Chords approximate the arc (a parabola sags below its chords),
 * so the per-cell Y band is approximate; resolution is controlled by `num_spine`.
 */
function trajectoryHitsHeightfieldSpan(
    cfg: JumpLinkBuilderConfig,
    heightfields: Heightfield[],
    pa: Vec3,
    pb: Vec3,
    tra: Trajectory,
): boolean {
    const n = tra.num_spine;
    while (spine.length < n) spine.push(vec3.create());

    for (let i = 0; i < n; i++) tra.apply(spine[i], pa, pb, i / (n - 1));

    for (const hf of heightfields) {
        for (let seg = 0; seg < n - 1; seg++) {
            if (segmentHitsHeightfield(cfg, hf, spine[seg], spine[seg + 1])) return true;
        }
    }
    return false;
}

function dist2DSqr(a: Vec3, b: Vec3): number {
    const dx = a[0] - b[0];
    const dz = a[2] - b[2];
    return dx * dx + dz * dz;
}

const invalidIndices = new Set<number>();

export function sampleTrajectory(cfg: JumpLinkBuilderConfig, heightfields: Heightfield[], es: EdgeSampler) {
    const nsamples = es.start.gsamples.length;
    for (let i = 0; i < nsamples; ++i) {
        const ssmp = es.start.gsamples[i];
        for (const end of es.end) {
            const esmp = end.gsamples[i];

            if (!ssmp.validHeight || !esmp.validHeight) continue;

            if (trajectoryHitsHeightfieldSpan(cfg, heightfields, ssmp.p, esmp.p, es.trajectory)) continue;

            ssmp.validTrajectory = true;
            esmp.validTrajectory = true;
        }
    }

    //radius post process
    //With dense sampling (sampleSpacing <= agentRadius), assume that trajectories within a distance of agent radius to an invalid trajectory will also be invalid.

    const radiusSqr = cfg.agentRadius * cfg.agentRadius;
    for (const end of es.end) {
        invalidIndices.clear();
        for (let i = 0; i < nsamples; ++i) {
            const esmp1 = end.gsamples[i];
            if (esmp1.validTrajectory) continue;
            for (let j = 0; j < nsamples; ++j) {
                if (i === j) continue;
                const esmp2 = end.gsamples[j];
                if (!esmp2.validTrajectory) continue;
                if (dist2DSqr(esmp1.p, esmp2.p) > radiusSqr) continue;
                invalidIndices.add(j);
            }
        }
        for (const index of invalidIndices) {
            end.gsamples[index].validTrajectory = false;
        }
    }
}
