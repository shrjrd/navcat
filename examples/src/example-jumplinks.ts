import { type Vec3, vec3 } from 'mathcat';
import {
    addOffMeshConnection,
    createFindNearestPolyResult,
    FindNodePathResultFlags,
    findNearestPoly,
    findNodePath,
    findPath,
    findStraightPath,
    getNodeByRef,
    getNodeRefType,
    NodeType,
    OffMeshConnectionDirection,
    type QueryFilter,
} from 'navcat';
import {
    buildJumpLinks,
    buildJumpSegments,
    createClimbTrajectory,
    createJumpLinkHelper,
    createJumpTrajectory,
    type EdgeSampler,
    extractEdges,
    filterOverlappingLinks,
    generateTiledNavMesh,
    getNearbyHeightfields,
    initClimbDownSampler,
    initEdgeJumpSampler,
    type JumpLink,
    type JumpLinkBuilderConfig,
    JumpLinkFlag,
    sampleGroundNavMesh,
    sampleTrajectory,
    type TiledNavMeshInput,
    type TiledNavMeshOptions,
    type Trajectory,
} from 'navcat/blocks';
import {
    createNavMeshHelper,
    createNavMeshOffMeshConnectionsHelper,
    createNavMeshPolyHelper,
    //createSearchNodesHelper,
    getPositionsAndIndices,
    primitivesToThreeJS,
} from 'navcat/three';
import * as THREE from 'three';
import { LineGeometry, OrbitControls } from 'three/examples/jsm/Addons.js';
import { Line2 } from 'three/examples/jsm/lines/webgpu/Line2.js';
import { Line2NodeMaterial } from 'three/webgpu';
import { createExample } from './common/example-base';
import { createFlag } from './common/flag';
import { loadGLTF } from './common/load-gltf';

/* setup example scene */
const container = document.getElementById('root')!;
const { scene, camera, renderer } = await createExample(container);

camera.position.set(-2, 10, 10);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const navTestModel = await loadGLTF('./models/nav-test.glb');
scene.add(navTestModel.scene);

/* generate navmesh */
const walkableMeshes: THREE.Mesh[] = [];
scene.traverse((object) => {
    if (object instanceof THREE.Mesh) {
        walkableMeshes.push(object);
    }
});

const [positions, indices] = getPositionsAndIndices(walkableMeshes);

const navMeshInput: TiledNavMeshInput = {
    positions,
    indices,
};

const cellSize = 0.15;
const cellHeight = 0.15;

const tileSizeVoxels = 16;
const tileSizeWorld = tileSizeVoxels * cellSize;

const walkableRadiusWorld = 0.1;
const walkableRadiusVoxels = Math.ceil(walkableRadiusWorld / cellSize);
const walkableClimbWorld = 0.5;
const walkableClimbVoxels = Math.ceil(walkableClimbWorld / cellHeight);
const walkableHeightWorld = 0.25;
const walkableHeightVoxels = Math.ceil(walkableHeightWorld / cellHeight);
const walkableSlopeAngleDegrees = 45;

const borderSize = 4;
const minRegionArea = 8;
const mergeRegionArea = 20;

const maxSimplificationError = 1.3;
const maxEdgeLength = 12;

const maxVerticesPerPoly = 5;

const detailSampleDistanceVoxels = 6;
const detailSampleDistance = detailSampleDistanceVoxels < 0.9 ? 0 : cellSize * detailSampleDistanceVoxels;

const detailSampleMaxErrorVoxels = 1;
const detailSampleMaxError = cellHeight * detailSampleMaxErrorVoxels;

const navMeshConfig: TiledNavMeshOptions = {
    cellSize,
    cellHeight,
    tileSizeVoxels,
    tileSizeWorld,
    walkableRadiusWorld,
    walkableRadiusVoxels,
    walkableClimbWorld,
    walkableClimbVoxels,
    walkableHeightWorld,
    walkableHeightVoxels,
    walkableSlopeAngleDegrees,
    borderSize,
    minRegionArea,
    mergeRegionArea,
    maxSimplificationError,
    maxEdgeLength,
    maxVerticesPerPoly,
    detailSampleDistance,
    detailSampleMaxError,
};

console.time('generateTiledNavMesh');
const navMeshResult = generateTiledNavMesh(navMeshInput, navMeshConfig);
console.timeEnd('generateTiledNavMesh');
const navMesh = navMeshResult.navMesh;

/* off mesh connection types */
enum OffMeshConnectionAreaType {
    TELEPORTER = 1,
    JUMP = 2,
    CLIMB = 3,
}

/* query filter */
const queryFilter: QueryFilter = {
    passFilter: (_nodeRef, _navMesh) => {
        return true;
    },
    getCost: (pa, pb, navMesh, _prevRef, curRef, _nextRef) => {
        // define the costs for traversing an off mesh connection
        if (curRef !== undefined && getNodeRefType(curRef) === NodeType.OFFMESH) {
            const { area } = getNodeByRef(navMesh, curRef);

            if (area === OffMeshConnectionAreaType.JUMP) {
                // regular distance
                return vec3.distance(pa, pb);
            } else if (area === OffMeshConnectionAreaType.CLIMB) {
                // distance * 4, big penalty
                return vec3.distance(pa, pb) * 4;
            } else if (area === OffMeshConnectionAreaType.TELEPORTER) {
                // low flat cost
                return 1;
            }
        }

        return vec3.distance(pa, pb);
    },
};

/* add off mesh connections */

interface Config extends JumpLinkBuilderConfig {
    area: OffMeshConnectionAreaType;
    flags: number;
    sampleEdge: (cfg: Config, sp: Vec3, sq: Vec3, trajectory: Trajectory) => EdgeSampler<Trajectory>;
    trajectory: Trajectory;
}

const ClimbConfig: Config = {
    agentRadius: walkableRadiusWorld,
    agentHeight: walkableHeightWorld,
    agentClimb: walkableClimbWorld,
    groundTolerance: cellSize * 2,
    startDistance: -walkableRadiusWorld * 0.5,
    endDistance: 0.4,
    minHeight: -4.2,
    heightRange: 0.0 - -4.2,
    sampleSpacing: cellSize * 1,
    minDistance: undefined!,
    area: OffMeshConnectionAreaType.CLIMB,
    flags: 0xffffff,
    sampleEdge: initClimbDownSampler,
    trajectory: createClimbTrajectory(),
};

const JumpConfig: Config = {
    agentRadius: walkableRadiusWorld,
    agentHeight: walkableHeightWorld,
    agentClimb: walkableClimbWorld,
    groundTolerance: cellSize * 2,
    startDistance: -walkableRadiusWorld * 0.2,
    endDistance: 2.4,
    minHeight: -3.0,
    heightRange: 0.3 - -3.0,
    sampleSpacing: cellSize * 1,
    minDistance: walkableRadiusWorld * 1,
    area: OffMeshConnectionAreaType.JUMP,
    flags: 0xffffff,
    sampleEdge: initEdgeJumpSampler,
    trajectory: createJumpTrajectory(0.3),
};

const configs: Config[] = [ClimbConfig, JumpConfig];

const JumpLinks: JumpLink<Config>[] = [];

function onJumpLinkBuilt(link: JumpLink<Config>) {
    JumpLinks.push(link);
}

const minEdgeLength = 0;

function onValidEdge(sp: Vec3, sq: Vec3) {
    if (vec3.distance(sp, sq) < minEdgeLength) return;
    for (const cfg of configs) {
        const es = cfg.sampleEdge(cfg, sp, sq, cfg.trajectory);
        sampleGroundNavMesh(cfg, navMesh, es, cellSize);
        sampleTrajectory(cfg, getNearbyHeightfields(es, navMeshResult.intermediates.heightfield), es);
        buildJumpLinks(cfg, es, buildJumpSegments(cfg, es), onJumpLinkBuilt);
    }
}

const buildJumpLinksStart = performance.now();

for (const [_key, tile] of Object.entries(navMesh.tiles)) {
    extractEdges(navMesh, tile, onValidEdge);
}

const buildJumpLinksTime = performance.now() - buildJumpLinksStart;

function onLinkOverlap(li: JumpLink<Config>, lj: JumpLink<Config>, spi: Vec3, sqi: Vec3, spj: Vec3, sqj: Vec3) {
    if (li.cfg.area !== lj.cfg.area) return true;

    if (vec3.squaredDistance(spi, sqi) > vec3.squaredDistance(spj, sqj)) {
        lj.linkFlags = JumpLinkFlag.FILTERED;
        return true;
    } else {
        li.linkFlags = JumpLinkFlag.FILTERED;
        return false;
    }
}

const filterOverlappingLinksStart = performance.now();
filterOverlappingLinks(JumpLinks, 0.5, onLinkOverlap);
const filterOverlappingLinksTime = performance.now() - filterOverlappingLinksStart;

const offMeshConnectionRadius = walkableRadiusWorld;
const minJumpHeight = 0;
const maxJumpHeight = 1.4;
const minJumpDistance = 0;
const maxJumpDistance = 1.5;
const maxDistanceDifference = 0.6;
const minSampleDistance = cellSize * 1;

const halfExtentsSample = vec3.fromValues(walkableRadiusWorld, walkableHeightWorld * 0.5, walkableRadiusWorld);
const queryFilterNoOffMesh: QueryFilter = {
    passFilter: (nodeRef) => {
        if (getNodeRefType(nodeRef) === NodeType.OFFMESH) return false;
        return queryFilter.passFilter(nodeRef, navMesh);
    },
    getCost: queryFilter.getCost,
};

function processSample(link: JumpLink<Config>, start: Vec3, end: Vec3) {
    const jumpHeight = Math.abs(start[1] - end[1]);
    const jumpDistance = vec3.distance(start, end);

    if (jumpHeight < minJumpHeight || jumpDistance < minJumpDistance) {
        return;
    }

    // Path check
    // If there exists a path between start and end with similar distance within maxDistanceDifference, don't create an off-mesh connection
    // Helps reduce jumps over small holes and jumps parallel to nearby edges in the navmesh

    const startNearestPolyResult = findNearestPoly(
        createFindNearestPolyResult(),
        navMesh,
        start,
        halfExtentsSample,
        queryFilterNoOffMesh,
    );

    const endNearestPolyResult = findNearestPoly(
        createFindNearestPolyResult(),
        navMesh,
        end,
        halfExtentsSample,
        queryFilterNoOffMesh,
    );

    const nodePathResult = findNodePath(
        navMesh,
        startNearestPolyResult.nodeRef,
        endNearestPolyResult.nodeRef,
        startNearestPolyResult.position,
        endNearestPolyResult.position,
        queryFilterNoOffMesh,
    );

    const isPathComplete = (nodePathResult.flags & FindNodePathResultFlags.COMPLETE_PATH) !== 0;
    if (isPathComplete) {
        const straightPathResult = findStraightPath(
            navMesh,
            startNearestPolyResult.position,
            endNearestPolyResult.position,
            nodePathResult.path,
        );

        let pathLength = 0;
        for (let i = 1; i < straightPathResult.path.length; i++) {
            pathLength += vec3.distance(straightPathResult.path[i - 1].position, straightPathResult.path[i].position);
        }

        if (Math.abs(pathLength - vec3.distance(start, end)) <= maxDistanceDifference) return;
    }

    let direction = OffMeshConnectionDirection.START_TO_END;
    if (jumpHeight <= maxJumpHeight && jumpDistance <= maxJumpDistance) {
        direction = OffMeshConnectionDirection.BIDIRECTIONAL;
    }

    return addOffMeshConnection(navMesh, {
        start,
        end,
        direction,
        radius: offMeshConnectionRadius,
        area: link.cfg.area,
        flags: link.cfg.flags,
    });
}

const totalNumberOfJumpLinks = Object.values(JumpLinks).length;
let unfilteredCount = 0;
let processJumpLinkTime = 0;
const prev = vec3.create();
function processJumpLink(link: JumpLink<Config>) {
    if (link.linkFlags === JumpLinkFlag.FILTERED) return;
    unfilteredCount++;
    let validSampleCount = 0;
    const processJumpLinkStart = performance.now();
    vec3.set(prev, Infinity, Infinity, Infinity);
    for (let i = 0; i < link.startSamples.length; i++) {
        const [startP, endP] = [link.startSamples[i].p, link.endSamples[i].p];
        // adjusting off mesh connection density separate from sampling density
        if (i === 0 || vec3.distance(prev, startP) >= minSampleDistance) {
            if (processSample(link, startP, endP)) {
                vec3.copy(prev, startP);
                validSampleCount++;
            }
        }
    }
    processJumpLinkTime += performance.now() - processJumpLinkStart;
    if (validSampleCount === 0) return;

    const jumpLinkHelper = primitivesToThreeJS(createJumpLinkHelper(link)).object;
    jumpLinkHelper.position.y += 0.1;
    scene.add(jumpLinkHelper);
}

for (const link of JumpLinks) processJumpLink(link);

/* create debug helpers */
const navMeshHelper = createNavMeshHelper(navMesh);
navMeshHelper.object.position.y += 0.1;
scene.add(navMeshHelper.object);

const offMeshConnectionsHelper = createNavMeshOffMeshConnectionsHelper(navMesh);
scene.add(offMeshConnectionsHelper.object);

/* find path */
let start: Vec3 = [-2.2, 0.26, 4.71];
let end: Vec3 = [3.4, 2.8, 3.6];
const halfExtents: Vec3 = [1, 1, 1];

type Visual = { object: THREE.Object3D; dispose: () => void };
let visuals: Visual[] = [];

function clearVisuals() {
    for (const visual of visuals) {
        scene.remove(visual.object);
        visual.dispose();
    }
    visuals = [];
}

function addVisual(visual: Visual) {
    visuals.push(visual);
    scene.add(visual.object);
}

function updatePath() {
    clearVisuals();

    const startFlag = createFlag(0x2196f3);
    startFlag.object.position.set(...start);
    addVisual(startFlag);

    const endFlag = createFlag(0x00ff00);
    endFlag.object.position.set(...end);
    addVisual(endFlag);

    console.time('findPath');
    const pathResult = findPath(navMesh, start, end, halfExtents, queryFilter);
    console.timeEnd('findPath');

    if (pathResult.success) {
        const { path, nodePath } = pathResult;

        if (nodePath) {
            /*
            const searchNodesHelper = createSearchNodesHelper(nodePath.nodes);
            addVisual({
                object: searchNodesHelper.object,
                dispose: () => {
                    // searchNodesHelper has its own disposal handled elsewhere; remove only
                },
            });
            */
            for (let i = 0; i < nodePath.path.length; i++) {
                const node = nodePath.path[i];

                if (getNodeRefType(node) === NodeType.POLY) {
                    const polyHelper = createNavMeshPolyHelper(navMesh, node);
                    polyHelper.object.position.y += 0.15;
                    addVisual({
                        object: polyHelper.object,
                        dispose: () => {
                            polyHelper.object.traverse((child) => {
                                if ((child as any).geometry) (child as any).geometry.dispose?.();
                                if ((child as any).material) {
                                    const mat = (child as any).material;
                                    if (Array.isArray(mat)) {
                                        mat.forEach((m: any) => {
                                            m?.dispose?.();
                                        });
                                    } else {
                                        mat.dispose?.();
                                    }
                                }
                            });
                        },
                    });
                }
            }
        }

        if (path) {
            for (let i = 0; i < path.length; i++) {
                const point = path[i];

                // point
                const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.2), new THREE.MeshBasicMaterial({ color: 0xff0000 }));
                mesh.position.set(...point.position);
                addVisual({
                    object: mesh,
                    dispose: () => {
                        mesh.geometry?.dispose();
                        (mesh.material as any)?.dispose?.();
                    },
                });

                // line
                if (i > 0) {
                    const prevPoint = path[i - 1];

                    const geometry = new LineGeometry();
                    geometry.setFromPoints([new THREE.Vector3(...prevPoint.position), new THREE.Vector3(...point.position)]);

                    const material = new Line2NodeMaterial({
                        color: 'yellow',
                        linewidth: 0.1,
                        worldUnits: true,
                    });

                    const line = new Line2(geometry, material);

                    addVisual({
                        object: line,
                        dispose: () => {
                            line.geometry?.dispose();
                            line.material?.dispose?.();
                        },
                    });
                }
            }
        }
    }

    updateStats();
}

/* interaction */
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
function getPointOnNavMesh(event: PointerEvent): Vec3 | null {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const intersects = raycaster.intersectObjects(walkableMeshes, true);
    if (intersects.length > 0) {
        const p = intersects[0].point;
        return [p.x, p.y, p.z];
    }
    return null;
}

let moving: 'start' | 'end' | null = null;

renderer.domElement.addEventListener('pointerdown', (event: PointerEvent) => {
    event.preventDefault();
    const point = getPointOnNavMesh(event);

    if (!point) return;

    if (event.button === 0) {
        if (moving === 'start') {
            moving = null;
            renderer.domElement.style.cursor = '';
            start = point;
        } else {
            moving = 'start';
            renderer.domElement.style.cursor = 'crosshair';
            start = point;
        }
    } else if (event.button === 2) {
        if (moving === 'end') {
            moving = null;
            renderer.domElement.style.cursor = '';
            end = point;
        } else {
            moving = 'end';
            renderer.domElement.style.cursor = 'crosshair';
            end = point;
        }
    }
    updatePath();
});

renderer.domElement.addEventListener('pointermove', (event: PointerEvent) => {
    if (!moving) return;

    const point = getPointOnNavMesh(event);
    if (!point) return;

    if (moving === 'start') {
        start = point;
    } else if (moving === 'end') {
        end = point;
    }

    updatePath();
});

renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

/* path stats */
const statsDiv = document.createElement('div');
statsDiv.style.position = 'absolute';
statsDiv.style.top = '10px';
statsDiv.style.left = '10px';
statsDiv.style.color = 'white';
statsDiv.style.fontFamily = 'monospace';
statsDiv.style.fontSize = '11px';
statsDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
statsDiv.style.padding = '10px';
statsDiv.style.borderRadius = '4px';
statsDiv.style.minWidth = '200px';
container.appendChild(statsDiv);

function updateStats() {
    let html = `<div style="margin-bottom: 8px; font-weight: bold; color: #00aaff;">Path Stats</div>`;

    // Start position
    html += `<div style="margin-bottom: 4px;">`;
    html += `<div style="color: #2196f3; font-weight: bold;">Start Position</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">X: ${start[0].toFixed(2)}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Y: ${start[1].toFixed(2)}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Z: ${start[2].toFixed(2)}</div>`;
    html += `</div>`;

    // End position
    html += `<div style="margin-bottom: 4px;">`;
    html += `<div style="color: #00ff00; font-weight: bold;">End Position</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">X: ${end[0].toFixed(2)}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Y: ${end[1].toFixed(2)}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Z: ${end[2].toFixed(2)}</div>`;
    html += `</div>`;

    // Path stats
    /*
    const pathResult = findPath(navMesh, start, end, halfExtents, queryFilter);

    html += `<div style="margin-top: 8px; padding-top: 8px; border-top: 1px solid rgba(255,255,255,0.2);">`;
    html += `<div style="color: #ffff00; font-weight: bold; margin-bottom: 4px;">Path Info</div>`;
    html += `<pre style="color: #ccc; font-size: 10px; margin: 0; overflow-x: auto; max-height: 400px; overflow-y: auto;">${JSON.stringify(pathResult, null, 2)}</pre>`;
    html += `</div>`;
    */

    // JumpLink stats
    html += `<div style="margin-top: 8px; padding-top: 8px; border-top: 1px solid rgba(255,255,255,0.2);">`;
    html += `<div style="color:rgb(255, 111, 0); font-weight: bold; margin-bottom: 4px;">JumpLink Stats</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">buildJumpLinks: ${(buildJumpLinksTime).toFixed(2)} ms</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Initial Jumplinks: ${totalNumberOfJumpLinks}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">filterOverlappingLinks: ${(filterOverlappingLinksTime).toFixed(2)} ms</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">Valid Jumplinks: ${unfilteredCount}</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">processJumpLink: ${(processJumpLinkTime).toFixed(2)} ms</div>`;
    html += `<div style="color: #ccc; padding-left: 8px;">OffMeshConnections: ${Object.values(navMesh.offMeshConnections).length}</div>`;
    html += `</div>`;

    statsDiv.innerHTML = html;
}

/* initial update */
updateStats();
updatePath();

/* start loop */
function update() {
    requestAnimationFrame(update);

    orbitControls.update();
    renderer.render(scene, camera);
}

update();
