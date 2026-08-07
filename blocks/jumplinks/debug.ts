import { type DebugPrimitive, DebugPrimitiveType } from 'navcat';
import type { EdgeSampler, GroundSample, GroundSegment, JumpLink } from './types';

export const createGroundSampleHelper = (groundSample: GroundSample): DebugPrimitive[] => {
    const primitives: DebugPrimitive[] = [];

    primitives.push({
        type: DebugPrimitiveType.Points,
        positions: [groundSample.p[0], groundSample.p[1], groundSample.p[2]],
        colors: [1, 1, 1],
        size: 0.025,
        transparent: true,
    });

    return primitives;
};

export const createGroundSegmentHelper = (groundSegment: GroundSegment): DebugPrimitive[] => {
    const primitives: DebugPrimitive[] = [];

    primitives.push({
        type: DebugPrimitiveType.Lines,
        positions: [
            groundSegment.p[0],
            groundSegment.p[1],
            groundSegment.p[2],
            groundSegment.q[0],
            groundSegment.q[1],
            groundSegment.q[2],
        ],
        colors: [1, 1, 1, 1, 1, 1],
        lineWidth: 0.025,
    });

    //const groundSamples = groundSegment.gsamples;
    //for (const groundSample of groundSamples)
    //primitives.push(...createGroundSampleHelper(groundSample));

    /*
    const p0 = groundSamples[0].p;
    const p1 = groundSamples[groundSamples.length - 1].p;
    primitives.push({
        type: DebugPrimitiveType.Lines,
        positions: [p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]],
        colors: [1, 1, 1, 1, 1, 1],
        lineWidth: 0.025,
    });
    */
    return primitives;
};

export const createEdgeSamplerHelper = (es: EdgeSampler): DebugPrimitive[] => {
    const primitives: DebugPrimitive[] = [];

    primitives.push(...createGroundSegmentHelper(es.start));

    for (const groundSegment of es.end) primitives.push(...createGroundSegmentHelper(groundSegment));

    return primitives;
};

const pushTrajLine = (
    positions: number[],
    colors: number[],
    x0: number,
    y0: number,
    z0: number,
    x1: number,
    y1: number,
    z1: number,
    c: [number, number, number],
) => {
    positions.push(x0, y0, z0, x1, y1, z1);
    colors.push(c[0], c[1], c[2], c[0], c[1], c[2]);
};

/** Filled ribbon between spine0/spine1, thin trajectory lines (spines + cross-ribs), and ground polylines. */
export const createJumpLinkHelper = (jumpLink: JumpLink): DebugPrimitive[] => {
    const primitives: DebugPrimitive[] = [];

    const spineCount = Math.min(jumpLink.nspine, Math.floor(jumpLink.spine0.length / 3), Math.floor(jumpLink.spine1.length / 3));
    if (spineCount < 2) {
        return primitives;
    }

    const trajColor: [number, number, number] = [38 / 255, 191 / 255, 1];
    const ribColor: [number, number, number] = [24 / 255, 120 / 255, 160 / 255];
    const groundColor: [number, number, number] = [100 / 255, 230 / 255, 90 / 255];

    const triPositions: number[] = [];
    const triColors: number[] = [];
    const triIndices: number[] = [];

    for (let j = 0; j < spineCount; j++) {
        const i = j * 3;
        triPositions.push(jumpLink.spine0[i], jumpLink.spine0[i + 1], jumpLink.spine0[i + 2]);
        triColors.push(trajColor[0], trajColor[1], trajColor[2]);
        triPositions.push(jumpLink.spine1[i], jumpLink.spine1[i + 1], jumpLink.spine1[i + 2]);
        triColors.push(trajColor[0], trajColor[1], trajColor[2]);
    }

    for (let j = 0; j < spineCount - 1; j++) {
        const base = j * 2;
        triIndices.push(base, base + 1, base + 2);
        triIndices.push(base + 1, base + 3, base + 2);
    }

    primitives.push({
        type: DebugPrimitiveType.Triangles,
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: true,
        opacity: 0.5,
        doubleSided: false,
    });

    const trajLinePositions: number[] = [];
    const trajLineColors: number[] = [];

    for (let j = 0; j < spineCount - 1; j++) {
        const a = j * 3;
        const b = (j + 1) * 3;
        pushTrajLine(
            trajLinePositions,
            trajLineColors,
            jumpLink.spine0[a],
            jumpLink.spine0[a + 1],
            jumpLink.spine0[a + 2],
            jumpLink.spine0[b],
            jumpLink.spine0[b + 1],
            jumpLink.spine0[b + 2],
            trajColor,
        );
        pushTrajLine(
            trajLinePositions,
            trajLineColors,
            jumpLink.spine1[a],
            jumpLink.spine1[a + 1],
            jumpLink.spine1[a + 2],
            jumpLink.spine1[b],
            jumpLink.spine1[b + 1],
            jumpLink.spine1[b + 2],
            trajColor,
        );
    }

    for (let j = 0; j < spineCount; j++) {
        const i = j * 3;
        pushTrajLine(
            trajLinePositions,
            trajLineColors,
            jumpLink.spine0[i],
            jumpLink.spine0[i + 1],
            jumpLink.spine0[i + 2],
            jumpLink.spine1[i],
            jumpLink.spine1[i + 1],
            jumpLink.spine1[i + 2],
            ribColor,
        );
    }

    if (trajLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: trajLinePositions,
            colors: trajLineColors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 1.25,
        });
    }

    const groundLinePositions: number[] = [];
    const groundLineColors: number[] = [];

    const pushGroundLine = (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number) => {
        groundLinePositions.push(x0, y0, z0, x1, y1, z1);
        groundLineColors.push(groundColor[0], groundColor[1], groundColor[2], groundColor[0], groundColor[1], groundColor[2]);
    };

    const startSamples = jumpLink.startSamples;
    for (let s = 0; s < startSamples.length - 1; s++) {
        const p0 = startSamples[s].p;
        const p1 = startSamples[s + 1].p;
        pushGroundLine(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
    }

    const endSamples = jumpLink.endSamples;
    for (let s = 0; s < endSamples.length - 1; s++) {
        const p0 = endSamples[s].p;
        const p1 = endSamples[s + 1].p;
        pushGroundLine(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
    }

    if (groundLinePositions.length > 0) {
        primitives.push({
            type: DebugPrimitiveType.Lines,
            positions: groundLinePositions,
            colors: groundLineColors,
            transparent: true,
            opacity: 0.5,
            lineWidth: 2,
        });
    }

    return primitives;
};
