import { type Vec3, vec2, vec3 } from 'mathcat';
import type { EdgeSampler, GroundSegment, JumpLinkBuilderConfig, Trajectory } from './types';

function trans2d(dst: number[], ax: number[], ay: number[], pt: number[]): void {
    dst[0] = ax[0] * pt[0] + ay[0] * pt[1];
    dst[1] = ax[1] * pt[0] + ay[1] * pt[1];
    dst[2] = ax[2] * pt[0] + ay[2] * pt[1];
}

function createEdgeSampler<T extends Trajectory>(sp: Vec3, sq: Vec3, trajectory: T): EdgeSampler<T> {
    const start: GroundSegment = {
        p: vec3.create(),
        q: vec3.create(),
        gsamples: [],
        height: 0,
    };
    const end: GroundSegment[] = [];
    const ax = vec3.create();
    const ay = vec3.create();
    const az = vec3.create();
    vec3.sub(ax, sq, sp);
    vec3.normalize(ax, ax);
    vec3.set(az, ax[2], 0, -ax[0]);
    vec3.normalize(az, az);
    vec3.set(ay, 0, 1, 0);
    return {
        start,
        end,
        trajectory,
        ax,
        ay,
        az,
    };
}

const offset = vec3.create();
const pt = vec2.create();

/** has multiple ground segments */
export function initEdgeJumpSampler<T extends Trajectory>(
    cfg: JumpLinkBuilderConfig,
    sp: Vec3,
    sq: Vec3,
    trajectory: T,
): EdgeSampler<T> {
    vec2.set(pt, cfg.startDistance, -cfg.agentClimb);
    const es = createEdgeSampler(sp, sq, trajectory);

    es.start.height = cfg.agentClimb * 2;
    trans2d(offset, es.az, es.ay, pt);
    vec3.add(es.start.p, sp, offset);
    vec3.add(es.start.q, sq, offset);

    const dx = cfg.endDistance - cfg.minDistance;
    const nsamples = Math.max(2, Math.ceil(dx / cfg.sampleSpacing));

    for (let j = 0; j < nsamples; ++j) {
        const v = j / (nsamples - 1);
        const ox = cfg.minDistance + dx * v;
        vec2.set(pt, ox, cfg.minHeight);
        trans2d(offset, es.az, es.ay, pt);
        const end: GroundSegment = {
            p: vec3.create(),
            q: vec3.create(),
            gsamples: [],
            height: 0,
        };
        end.height = cfg.heightRange;
        vec3.add(end.p, sp, offset);
        vec3.add(end.q, sq, offset);
        es.end.push(end);
    }
    return es;
}

/** has one ground segment */
export function initClimbDownSampler<T extends Trajectory>(
    cfg: JumpLinkBuilderConfig,
    sp: Vec3,
    sq: Vec3,
    trajectory: T,
): EdgeSampler<T> {
    vec2.set(pt, cfg.startDistance, -cfg.agentClimb);
    const es = createEdgeSampler(sp, sq, trajectory);

    es.start.height = cfg.agentClimb * 2;
    trans2d(offset, es.az, es.ay, pt);
    vec3.add(es.start.p, sp, offset);
    vec3.add(es.start.q, sq, offset);

    vec2.set(pt, cfg.endDistance, cfg.minHeight);
    trans2d(offset, es.az, es.ay, pt);
    const end: GroundSegment = {
        p: vec3.create(),
        q: vec3.create(),
        gsamples: [],
        height: cfg.heightRange,
    };
    vec3.add(end.p, sp, offset);
    vec3.add(end.q, sq, offset);
    es.end.push(end);
    return es;
}