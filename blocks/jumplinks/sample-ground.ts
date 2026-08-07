import { type Box3, box3, type Vec3, vec3 } from 'mathcat';
import { ANY_QUERY_FILTER, createGetPolyHeightResult, getNodeByRef, getPolyHeight, type NavMesh, queryPolygons } from 'navcat';
import type { EdgeSampler, GroundSample, GroundSegment, JumpLinkBuilderConfig } from './types';

function vDist2DSqr(v1: Vec3, v2: Vec3): number {
    const dx = v2[0] - v1[0];
    const dz = v2[2] - v1[2];
    return dx * dx + dz * dz;
}

const _pt = vec3.create();

function sampleGroundSegment(heightFunc: (pt: Vec3, h: number) => [boolean, number], seg: GroundSegment, nsamples: number): void {
    seg.gsamples = new Array<GroundSample>(nsamples);

    for (let i = 0; i < nsamples; ++i) {
        const u = i / (nsamples - 1);

        const s: GroundSample = {
            p: vec3.create(),
            validTrajectory: false,
            validHeight: false,
        };
        seg.gsamples[i] = s;
        vec3.lerp(_pt, seg.p, seg.q, u);
        const height: [boolean, number] = heightFunc(_pt, seg.height);
        vec3.set(s.p, _pt[0], height[1], _pt[2]);

        if (!height[0]) {
            continue;
        }
        s.validHeight = true;
    }
}

function sampleGroundAbstract(
    cfg: JumpLinkBuilderConfig,
    es: EdgeSampler,
    heightFunc: (pt: Vec3, h: number) => [boolean, number],
): void {
    const dist = Math.sqrt(vDist2DSqr(es.start.p, es.start.q));
    const ngsamples = Math.max(2, Math.ceil(dist / cfg.sampleSpacing));
    sampleGroundSegment(heightFunc, es.start, ngsamples);
    for (const end of es.end) {
        sampleGroundSegment(heightFunc, end, ngsamples);
    }
}

const getPolyHeightResult = createGetPolyHeightResult();
const halfExtents = vec3.create();
const minV = vec3.create();
const maxV = vec3.create();
const bounds: Box3 = [0, 0, 0, 0, 0, 0];

function getNavMeshHeight(nav: NavMesh, pt: Vec3, heightRange: number, cellSize: number): [boolean, number] {
    vec3.set(halfExtents, cellSize, heightRange, cellSize);

    vec3.sub(minV, pt, halfExtents);
    vec3.add(maxV, pt, halfExtents);
    box3.set(bounds, minV[0], minV[1], minV[2], maxV[0], maxV[1], maxV[2]);

    const maxHeight = pt[1] + heightRange;
    let found = false;
    let minHeight = pt[1];

    const polys = queryPolygons(nav, bounds, ANY_QUERY_FILTER);
    for (let i = 0; i < polys.length; i++) {
        const ref = polys[i];
        const node = getNodeByRef(nav, ref);
        if (!node) continue;
        const tile = nav.tiles[node.tileId];
        if (!tile) continue;
        const poly = tile.polys[node.polyIndex];
        if (!poly) continue;

        getPolyHeight(getPolyHeightResult, tile, poly, node.polyIndex, pt);
        if (!getPolyHeightResult.success) continue;
        const y = getPolyHeightResult.height;
        if (y > minHeight && y < maxHeight) {
            found = true;
            minHeight = y;
        }
    }

    return [found, minHeight];
}

export function sampleGroundNavMesh(cfg: JumpLinkBuilderConfig, navMesh: NavMesh, es: EdgeSampler, cellSize: number): void {
    sampleGroundAbstract(cfg, es, (pt, h) => getNavMeshHeight(navMesh, pt, h, cellSize));
}
