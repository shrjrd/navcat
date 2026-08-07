import type { Vec3 } from 'mathcat';

/*

top down view

              sampleSpacing <->
sampled points on edge  ->  . . . . . . . .


            minDistance ->  . . . . . . . . <- GroundSegment
                            . . . . . . . . <- GroundSegment
                            . . . . . . . .
                            . . . . . . . .
            endDistance ->  . . . . . . . .


side view
sampled points on edge -> .


    minDistance, minHeight -----> . . . . . <- endDistance

*/

export interface JumpLinkBuilderConfig {
    readonly agentRadius: number;
    /** Maximum ledge height the agent can step; used when offsetting start/end segments and when grouping valid landing samples into jump segments. */
    readonly agentClimb: number;

    /** Extra vertical clearance above the ground when testing the trajectory against the heightfield (feet + tolerance). */
    readonly groundTolerance: number;

    /** Agent height; top of the trajectory for clearance tests against the heightfield. */
    readonly agentHeight: number;

    /** Distance along the edge outward normal from the boundary to place the take-off (start) ground segment. */
    readonly startDistance: number;

    /**
     * For jump sampling: far boundary of the landing strip sweep; columns are placed from {@link minDistance} to this value.
     * For climb-down sampling: distance along the outward normal to the single landing segment.
     */
    readonly endDistance: number;

    /** Vertical offset of the end ground segment(s) relative to the edge plane; lower bound of the landing search band. */
    readonly minHeight: number;

    /** Vertical range above {@link minHeight} (heightMax - heightMin)used when raying/snapping to ground for landing positions and nav mesh height queries. */
    readonly heightRange: number;

    /**
     * Target spacing used to derive sample count along ground segments and across the jump landing strip (column count).
     */
    readonly sampleSpacing: number;

    /** Near edge of the landing strip: minimum outward distance from the boundary to the closest landing segment. Forms the inner bound of the range swept to {@link endDistance}. */
    readonly minDistance: number;
}

/** A single candidate point on a ground segment, with validity flags filled in by the samplers. */
export interface GroundSample {
    /** Sample position in world space (snapped to ground height when {@link validHeight} is true). */
    p: Vec3;
    /** True when the arc between this sample and its counterpart on the opposite ground segment clears the heightfield. Set on both the start and end sample of a valid pair. */
    validTrajectory: boolean;
    /** True when the navmesh ground query at this xz location found a surface inside the search band. */
    validHeight: boolean;
}

/**
 * A line segment of candidate ground samples. For the start side this is the take-off strip along the edge;
 * for the end side it is either a landing strip (jump sampler) or a single landing segment (climb-down sampler).
 */
export interface GroundSegment {
    /** Segment start endpoint (world space). */
    p: Vec3;
    /** Segment end endpoint (world space). */
    q: Vec3;
    /** Samples along the segment from {@link p} to {@link q}; count is derived from the start segment's length divided by `sampleSpacing` from {@link JumpLinkBuilderConfig} and shared across all end segments. */
    gsamples: GroundSample[];
    /** Vertical search band passed to the ground height query: `agentClimb * 2` for start segments, `heightRange` for end segments. */
    height: number;
}

/** Parametric arc between a start and end point. Implementations define the shape (jump parabola, flat & straight climb-down). */
export interface Trajectory {
    /**
     * Evaluate the arc at parameter {@link u} ∈ [0, 1] between {@link start} and {@link end}.
     * Returns a newly allocated world-space position.
     */
    apply(out: Vec3, start: Vec3, end: Vec3, u: number): void;

    /**
     * number of points defining the polyline segments
     */
    num_spine: number;
}

/** Per-edge sampling state: the start/end ground segments plus the local frame used to place them. */
export interface EdgeSampler<trajectory extends Trajectory = Trajectory> {
    /** Take-off ground segment offset from the edge along the outward normal. */
    start: GroundSegment;
    /**
     * Candidate landing segments. The jump sampler emits one segment per column of the landing strip;
     * the climb-down sampler emits a single segment.
     */
    end: GroundSegment[];
    /** Arc used to connect paired start/end samples during trajectory validation and link spine generation. */
    trajectory: trajectory;
    /** Edge tangent: normalized direction from `edge.sp` to `edge.sq`. */
    ax: Vec3;
    /** World up axis (0, 1, 0). */
    ay: Vec3;
    /** Outward horizontal normal of the edge, orthogonal to {@link ax} in the xz plane. */
    az: Vec3;
}

/**
 * A contiguous run of valid paired samples identified during region flood-fill, before being promoted to a {@link JumpLink}.
 * Indices address the start ground segment's `gsamples`; the same index range is reused on the chosen end segment.
 */
export interface JumpSegment {
    /** Index into {@link EdgeSampler.end} selecting which landing segment this run pairs with. */
    groundSegment: number;
    /** First index into `EdgeSampler.start.gsamples` (and the corresponding end segment's `gsamples`) in the run. */
    startSample: number;
    /** Number of samples in the run, starting at {@link startSample}. */
    samples: number;
}

/** Validity state of a generated {@link JumpLink} after the overlap filter pass. */
export enum JumpLinkFlag {
    /** Link was suppressed by {@link filterOverlappingLinks}; should be ignored when consuming results. */
    FILTERED = 0,
    /** Link survived filtering and should be consumed. */
    VALID = 1,
}

/**
 * An emitted jump connection between two navmesh regions, expressed as two parallel spines.
 * Each spine is a flat `[x, y, z, x, y, z, ...]` buffer of {@link nspine} points
 */
export interface JumpLink<Config extends JumpLinkBuilderConfig = JumpLinkBuilderConfig> {
    /** Number of points per spine (equal to the generating trajectory's `num_spine`). */
    nspine: number;
    /** Flat xyz buffer for the spine along the left edge of the ribbon, length `nspine * 3`. */
    spine0: number[];
    /** Flat xyz buffer for the spine along the right edge of the ribbon, length `nspine * 3`. */
    spine1: number[];
    /** Contiguous slice of start-side ground samples that seeded this link. */
    startSamples: GroundSample[];
    /** Contiguous slice of end-side ground samples paired with {@link startSamples} (same indices). */
    endSamples: GroundSample[];
    /** Take-off ground segment this link was generated from. */
    start: GroundSegment;
    /** Landing ground segment this link was generated from. */
    end: GroundSegment;
    /** Configuration used to generate this link; retains any caller-defined extras from {@link Config}. */
    cfg: Config;
    /** Filter state; links marked {@link JumpLinkFlag.FILTERED} should be skipped by consumers. */
    linkFlags: JumpLinkFlag;
}
