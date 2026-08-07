import type { Vec3 } from 'mathcat';
import type { Heightfield } from 'navcat';
import type { EdgeSampler } from './types';

// This is only relevant for tiled navmeshes when jump trajectories go across multiple tiles.

/**
 * Collect the heightfields a sampler's trajectories could pass through, used as the broad phase for
 * {@link sampleTrajectory}'s collision test.
 *
 * Selection is by XZ overlap against each heightfield's own (border-expanded) bounds, not by mapping
 * the segment AABB onto the un-expanded tile grid. The grid approach missed obstacles whose voxels
 * live in a tile's border region — that geometry sits in the heightfield's expanded bounds but the
 * grid cell it physically overlaps belongs to a neighbouring tile, so the arc tested the wrong
 * heightfield and clipped through. Testing real bounds avoids that, and naturally handles a
 * trajectory spanning several tiles.
 *
 * The take-off/landing segment AABB bounds the arc in XZ (the jump trajectory is XZ-linear between
 * paired samples); it is inflated by one heightfield cell as a safety margin for arcs that graze a
 * heightfield edge.
 */
export function getNearbyHeightfields(es: EdgeSampler, heightfields: Heightfield[]) {
    let minX = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxZ = -Infinity;

    const accumulate = (v: Vec3) => {
        if (v[0] < minX) minX = v[0];
        if (v[0] > maxX) maxX = v[0];
        if (v[2] < minZ) minZ = v[2];
        if (v[2] > maxZ) maxZ = v[2];
    };

    accumulate(es.start.p);
    accumulate(es.start.q);
    for (const end of es.end) {
        accumulate(end.p);
        accumulate(end.q);
    }

    const result: Heightfield[] = [];
    for (const hf of heightfields) {
        // hf.bounds layout: [minX, minY, minZ, maxX, maxY, maxZ]; test XZ overlap only — the DDA in
        // sampleTrajectory handles Y per cell. Inflate by one cell so arcs grazing an edge still match.
        const m = hf.cellSize;
        if (hf.bounds[3] + m < minX || hf.bounds[0] - m > maxX) continue;
        if (hf.bounds[5] + m < minZ || hf.bounds[2] - m > maxZ) continue;
        result.push(hf);
    }

    return result;
}
