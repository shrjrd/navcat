import { vec3 } from 'mathcat';
import { type EdgeSampler, type JumpLink, type JumpLinkBuilderConfig, JumpLinkFlag, type JumpSegment } from './types';

const ppt = vec3.create();
const qpt = vec3.create();

export function buildJumpLinks<Config extends JumpLinkBuilderConfig = JumpLinkBuilderConfig>(
    cfg: Config,
    es: EdgeSampler,
    jumpSegments: JumpSegment[],
    callback: (link: JumpLink<Config>) => void,
) {
    const nspine = es.trajectory.num_spine;
    for (const js of jumpSegments) {
        const sp = es.start.gsamples[js.startSample].p;
        const sq = es.start.gsamples[js.startSample + js.samples - 1].p;
        const end = es.end[js.groundSegment];
        const ep = end.gsamples[js.startSample].p;
        const eq = end.gsamples[js.startSample + js.samples - 1].p;

        const from = js.startSample;
        const to = from + js.samples;
        const link: JumpLink<Config> = {
            nspine,
            spine0: new Array<number>(nspine * 3),
            spine1: new Array<number>(nspine * 3),
            startSamples: es.start.gsamples.slice(from, to),
            endSamples: end.gsamples.slice(from, to),
            start: es.start,
            end,
            cfg,
            linkFlags: JumpLinkFlag.VALID,
        };

        for (let j = 0; j < nspine; ++j) {
            const u = j / (nspine - 1);
            es.trajectory.apply(ppt, sp, ep, u);
            es.trajectory.apply(qpt, sq, eq, u);
            vec3.toBuffer(link.spine0, ppt, j * 3);
            vec3.toBuffer(link.spine1, qpt, j * 3);
        }

        callback(link);
    }
}
