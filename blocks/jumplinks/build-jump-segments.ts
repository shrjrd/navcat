import type { EdgeSampler, JumpLinkBuilderConfig, JumpSegment } from './types';

function addNeighbour(es: EdgeSampler, queue: number[][], agentClimb: number, h: number, i: number, j: number): void {
    const q = es.end[j].gsamples[i];
    if (q.validTrajectory && Math.abs(q.p[1] - h) < agentClimb) queue.push([i, j]);
}

function fill(es: EdgeSampler, sampleGrid: number[][], queue: number[][], head: number, agentClimb: number, region: number) {
    while (head < queue.length) {
        const ij: number[] = queue[head];
        head++;
        const i = ij[0];
        const j = ij[1];
        if (sampleGrid[i][j] === -1) {
            const p = es.end[j].gsamples[i];
            sampleGrid[i][j] = region;
            const h = p.p[1];
            if (i < sampleGrid.length - 1) {
                addNeighbour(es, queue, agentClimb, h, i + 1, j);
            }
            if (i > 0) {
                addNeighbour(es, queue, agentClimb, h, i - 1, j);
            }
            if (j < sampleGrid[0].length - 1) {
                addNeighbour(es, queue, agentClimb, h, i, j + 1);
            }
            if (j > 0) {
                addNeighbour(es, queue, agentClimb, h, i, j - 1);
            }
        }
    }
}

const sampleGrid: number[][] = [];

export function buildJumpSegments(cfg: JumpLinkBuilderConfig, es: EdgeSampler) {
    const n = es.end[0].gsamples.length;
    sampleGrid.length = n;
    for (let i = 0; i < n; i++)
        sampleGrid[i] = new Array<number>(es.end.length);

    for (let j = 0; j < es.end.length; j++) {
        for (let i = 0; i < n; i++)
            sampleGrid[i][j] = -1;
    }
    // Fill connected regions
    let region = 0;
    for (let j = 0; j < es.end.length; j++) {
        for (let i = 0; i < n; i++) {
            if (sampleGrid[i][j] === -1) {
                const p = es.end[j].gsamples[i];
                if (!p.validTrajectory) {
                    sampleGrid[i][j] = -2;
                } else {
                    const queue: number[][] = [[i, j]];
                    fill(es, sampleGrid, queue, 0, cfg.agentClimb, region);
                    region++;
                }
            }
        }
    }
    const jumpSegments = new Array<JumpSegment>(region);
    for (let i = 0; i < region; i++)
        jumpSegments[i] = {
            groundSegment: 0,
            startSample: 0,
            samples: 0,
        };

    // Find longest segments per region
    for (let j = 0; j < es.end.length; j++) {
        let l = 0;
        let r = -2;
        for (let i = 0; i < n + 1; i++) {
            if (i === n || sampleGrid[i][j] !== r) {
                if (r >= 0) {
                    if (jumpSegments[r].samples < l) {
                        jumpSegments[r].samples = l;
                        jumpSegments[r].startSample = i - l;
                        jumpSegments[r].groundSegment = j;
                    }
                }
                if (i < n) {
                    r = sampleGrid[i][j];
                }
                l = 1;
            } else {
                l++;
            }
        }
    }
    return jumpSegments;
}
