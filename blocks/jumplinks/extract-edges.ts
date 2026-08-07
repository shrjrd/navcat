import { type Vec3, vec3 } from 'mathcat';
import { getNodeByTileAndPoly, INVALID_NODE_REF, type NavMesh, type NavMeshTile, POLY_NEIS_FLAG_EXT_LINK } from 'navcat';

function addEdge(j: number, nv: number, polyVertices: number[], tileVertices: number[], callback: (sp: Vec3, sq: Vec3) => void) {
    const nj = (j + 1) % nv;
    const va = polyVertices[j] * 3;
    const vb = polyVertices[nj] * 3;
    const sp = vec3.fromBuffer(vec3.create(), tileVertices, vb);
    const sq = vec3.fromBuffer(vec3.create(), tileVertices, va);

    callback(sp, sq);
}

/**
 * Walks every border edge of a tile, invoking `callback` with the edge endpoints and the index of
 * the polygon that owns the edge. A border edge is a true border (`nei === 0`) or an unconnected
 * portal edge. Shared by {@link extractEdges} and {@link extractEdgePairs}.
 */
function forEachBorderEdge(navMesh: NavMesh, tile: NavMeshTile, callback: (sp: Vec3, sq: Vec3, polyIdx: number) => void) {
    const navMeshLinks = navMesh.links;

    const tileVertices = tile.vertices;
    const polys = tile.polys;
    for (let polyIdx = 0; polyIdx < polys.length; polyIdx++) {
        const poly = polys[polyIdx];
        const node = getNodeByTileAndPoly(navMesh, tile, polyIdx);
        const nodeLinks = node.links;
        const neis = poly.neis;
        const polyVertices = poly.vertices;
        const nv = polyVertices.length;
        for (let j = 0; j < nv; j++) {
            const nei = neis[j];
            if (nei === 0) {
                // True border edge: emit the full edge
                addEdge(j, nv, polyVertices, tileVertices, (sp, sq) => callback(sp, sq, polyIdx));
            } else if (nei & POLY_NEIS_FLAG_EXT_LINK) {
                // Portal edge: check if it's actually connected
                let hasLink = false;
                for (const linkIndex of nodeLinks) {
                    const link = navMeshLinks[linkIndex];
                    if (link && link.edge === j && link.toNodeRef !== INVALID_NODE_REF) {
                        hasLink = true;
                        break;
                    }
                }
                if (hasLink) continue;
                // Unconnected portal: this is also a border edge
                addEdge(j, nv, polyVertices, tileVertices, (sp, sq) => callback(sp, sq, polyIdx));
            }
            // else: internal edge, skip
        }
    }
}

export function extractEdges(navMesh: NavMesh, tile: NavMeshTile, callback: (sp: Vec3, sq: Vec3) => void) {
    forEachBorderEdge(navMesh, tile, (sp, sq) => callback(sp, sq));
}