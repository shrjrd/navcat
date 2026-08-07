import { type Vec3, vec3 } from 'mathcat';
import { geometry } from 'navcat';
import { type JumpLink, type JumpLinkBuilderConfig, JumpLinkFlag } from './types';

const spi = vec3.create();
const sqi = vec3.create();
const epi = vec3.create();
const eqi = vec3.create();
const spj = vec3.create();
const sqj = vec3.create();
const epj = vec3.create();
const eqj = vec3.create();

/**
 * Port of Unreal's `dtNavLinkBuilder::filterOverlappingLinks`
 *
 * For every pair of links `(i, j)`, when all four endpoint corners sit within the threshold, the narrower link is dropped.
 * Links with different `area` are kept (they produce different pathing cost).
 */
export function filterOverlappingLinks<Config extends JumpLinkBuilderConfig = JumpLinkBuilderConfig>(
    links: JumpLink<Config>[],
    edgeDistanceThreshold: number,
    callback: (
        li: JumpLink<Config>,
        lj: JumpLink<Config>,
        spi: Vec3,
        sqi: Vec3,
        spj: Vec3,
        sqj: Vec3,
        epi: Vec3,
        eqi: Vec3,
        epj: Vec3,
        eqj: Vec3,
    ) => boolean,
) {
    if (edgeDistanceThreshold <= 0) return;
    const thresholdSquared = edgeDistanceThreshold * edgeDistanceThreshold;
    for (let i = 0; i < links.length - 1; i++) {
        const li = links[i];
        if (li.linkFlags === JumpLinkFlag.FILTERED) continue;
        const endOffI = (li.nspine - 1) * 3;
        vec3.fromBuffer(spi, li.spine0, 0);
        vec3.fromBuffer(sqi, li.spine1, 0);
        vec3.fromBuffer(epi, li.spine0, endOffI);
        vec3.fromBuffer(eqi, li.spine1, endOffI);

        for (let j = i + 1; j < links.length; j++) {
            const lj = links[j];
            if (lj.linkFlags === JumpLinkFlag.FILTERED) continue;
            const endOffJ = (lj.nspine - 1) * 3;
            vec3.fromBuffer(spj, lj.spine0, 0);
            vec3.fromBuffer(sqj, lj.spine1, 0);
            vec3.fromBuffer(epj, lj.spine0, endOffJ);
            vec3.fromBuffer(eqj, lj.spine1, endOffJ);

            // Reverse overlap (Unreal's original): j's near corners vs i's far segment and j's far corners vs i's near segment.
            // Catches links that cover the same rectangle in opposite directions.
            const d0 = geometry.distancePtSeg(spj, epi, eqi);
            const d1 = geometry.distancePtSeg(sqj, epi, eqi);
            const d2 = geometry.distancePtSeg(epj, spi, sqi);
            const d3 = geometry.distancePtSeg(eqj, spi, sqi);
            const reverseOverlap =
                d0 < thresholdSquared && d1 < thresholdSquared && d2 < thresholdSquared && d3 < thresholdSquared;

            // Forward overlap (navcat addition, not in Unreal): near-vs-near and far-vs-far
            // Catches same-direction duplicates produced by similar configs
            const f0 = geometry.distancePtSeg(spj, spi, sqi);
            const f1 = geometry.distancePtSeg(sqj, spi, sqi);
            const f2 = geometry.distancePtSeg(epj, epi, eqi);
            const f3 = geometry.distancePtSeg(eqj, epi, eqi);
            const forwardOverlap =
                f0 < thresholdSquared && f1 < thresholdSquared && f2 < thresholdSquared && f3 < thresholdSquared;

            if (!reverseOverlap && !forwardOverlap) continue;

            const result = callback(li, lj, spi, sqi, spj, sqj, epi, eqi, epj, eqj);
            if (result === true) {
                continue;
            } else {
                break;
            }
        }
    }
}
