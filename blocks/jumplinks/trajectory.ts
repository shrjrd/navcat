import { lerp, type Vec3 } from 'mathcat';
import type { Trajectory } from './types';

export interface ClimbTrajectory extends Trajectory {}

function applyClimbTrajectory(out: Vec3, start: Vec3, end: Vec3, u: number) {
    out[0] = lerp(start[0], end[0], Math.min(2 * u, 1));
    out[1] = lerp(start[1], end[1], Math.max(0, 2 * u - 1));
    out[2] = lerp(start[2], end[2], Math.min(2 * u, 1));
}

/** Produces a trajectory that goes forward a little and then drops down */
export function createClimbTrajectory(): ClimbTrajectory {
    // 3 spine points: u = 0, 0.5, 1. The midpoint lands exactly on the corner where XZ
    // pins and Y starts dropping, so the two chords represent the two linear pieces exactly.
    return {
        apply: applyClimbTrajectory,
        num_spine: 3,
    };
}

function interpolateHeight(ys: number, ye: number, u: number, jumpHeight: number) {
    if (u === 0) {
        return ys;
    } else if (u === 1.0) {
        return ye;
    }
    let [h1, h2] = [0, 0]
    if (ys >= ye) {
        // jump down
        h1 = jumpHeight;
        h2 = jumpHeight + ys - ye;
    } else {
        // jump up
        h1 = jumpHeight + ys - ye;
        h2 = jumpHeight;
    }
    const t = Math.sqrt(h1) / (Math.sqrt(h2) + Math.sqrt(h1));
    if (u <= t) {
        const v = 1.0 - u / t;
        return ys + h1 - h1 * v * v;
    }
    const v = (u - t) / (1.0 - t);
    return ys + h1 - h2 * v * v;
}

export interface JumpTrajectory extends Trajectory {
    jumpHeight: number;
}

function applyJumpTrajectory(out: Vec3, start: Vec3, end: Vec3, u: number, jumpHeight: number) {
    out[0] = lerp(start[0], end[0], u);
    out[1] = interpolateHeight(start[1], end[1], u, jumpHeight);
    out[2] = lerp(start[2], end[2], u);
}

/** Produces a parabolic trajectory */
export function createJumpTrajectory(jumpHeight: number): JumpTrajectory {
    // 8 spine points (7 chords) approximate the parabola; chords sag slightly below the
    // arc near the apex, so the sampled Y band is approximate rather than exact.
    return {
        apply: (out: Vec3, start: Vec3, end: Vec3, u: number) => applyJumpTrajectory(out, start, end, u, jumpHeight),
        num_spine: 8,
        jumpHeight,
    };
}
