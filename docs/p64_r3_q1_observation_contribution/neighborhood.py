"""Write immutable-id arrays and deterministic local score statistics."""
from pathlib import Path
import sys
import time
import warnings
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import kurtosis, skew
from freeze import PAIRS, read, write

def run_stats(run,out):
    rows, timing = [], []
    fig,axs=plt.subplots(2,4,figsize=(13,6.3),layout='constrained')
    for column, sample in enumerate(read(out/'sample_manifest.csv')):
        tick=time.perf_counter()
        frame=sample['frame']
        base=run/f'{frame}_nominal'
        target=run/f'{frame}_perturb'
        xyz=lambda path: np.fromfile(path,'<f4').reshape(-1,3)
        source, raw_map = xyz(base/'source.bin'), xyz(base/'map.bin')
        transforms=np.load(run/f'{frame}_poses.npz')
        poses, nominal=transforms['poses'],transforms['nominal']
        def score(k,j):
            return np.fromfile((base if j in (0,13) else target)/f'{k}_{0 if j==13 else j}.bin','<f8').reshape(len(source),2)
        raw=np.stack([[score(k,j) for j in range(26)] for k in range(len(poses))])
        s=raw[:,:,:,0]
        assoc=raw[:,:,:,1]>0
        info=np.stack([np.fromfile(base/f'{k}_0_info.bin','<f8').reshape(len(source),6,6) for k in range(len(poses))])
        np.savez_compressed(run/f'{frame}.npz',source=source,map=raw_map,nominal=nominal,poses=poses,
                            score=s,associated=assoc,neighbor_count=raw[:,:,:,1].astype(np.uint8),info=info)
        stats={}
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',RuntimeWarning)
            for scale, ix in enumerate([slice(0,13),slice(13,26)],start=1):
                cloud=s[:,ix]
                def save(name,value):
                    stats[f's{scale}_{name}']=value
                save('mean',cloud.mean(axis=1))
                save('sd',cloud.std(axis=1,ddof=1))
                save('median',np.median(cloud,axis=1))
                save('q10',np.quantile(cloud,.1,axis=1))
                save('q90',np.quantile(cloud,.9,axis=1))
                save('skewness',skew(cloud,axis=1,bias=False))
                save('excess_kurtosis',kurtosis(cloud,axis=1,bias=False))
                save('association_fraction',assoc[:,ix].mean(axis=1))
                for name,a,b in PAIRS[frame]:
                    d=cloud[a]-cloud[b]
                    flip=((d[1:]*d[:1])<0).mean(axis=0)
                    save(f'{name}_flip',flip)
                    save(f'{name}_paired_sd',d.std(axis=0,ddof=1))
                    save(f'{name}_paired_q10',np.quantile(d,.1,axis=0))
                    save(f'{name}_paired_q90',np.quantile(d,.9,axis=0))
                    full=d.sum(axis=1)
                    rows.append(dict(frame=frame,pair=name,scale=scale,source_points=len(source),
                        fraction_point_any_flip=(flip>0).mean(),mean_point_flip=flip.mean(),
                        stable_association_a=(assoc[a,ix].mean(axis=0)==1).mean(),
                        stable_association_b=(assoc[b,ix].mean(axis=0)==1).mean(),
                        median_abs_skew_a=np.nanmedian(np.abs(stats[f's{scale}_skewness'][a])),
                        median_abs_skew_b=np.nanmedian(np.abs(stats[f's{scale}_skewness'][b])),
                        median_excess_kurtosis_a=np.nanmedian(stats[f's{scale}_excess_kurtosis'][a]),
                        median_excess_kurtosis_b=np.nanmedian(stats[f's{scale}_excess_kurtosis'][b]),
                        full_delta_min=full.min(),full_delta_max=full.max(),full_delta_a_wins=(full>0).sum()))
        np.savez_compressed(run/f'neighborhood_stats_{frame}.npz',**stats)
        d=s[0,:13]-s[1,:13]
        ax=axs[0,column]
        ax.hist(d[0],density=True,bins=32,histtype='step',label='Nominal')
        ax.hist(d.mean(axis=0),density=True,bins=32,histtype='step',label='Neighborhood mean')
        ax.legend(fontsize=6)
        ax.set_title(frame)
        ax=axs[1,column]
        ids=np.argsort(s[0,0]-s[1,0]); ids=ids[np.linspace(0,len(ids)-1,19).astype(int)]
        ax.boxplot(d[:,ids],showfliers=False)
        ax.axhline(0,color='gray',lw=.6)
        ax.set_xticks([])
        ax.set_xlabel('19 quantile-spaced source IDs')
        timing.append(dict(frame=frame,stage='cache_and_neighborhood_statistics',wall_s=time.perf_counter()-tick))
    axs[0,0].set_ylabel('Density / native reward units')
    axs[1,0].set_ylabel('Paired delta / native reward units')
    fig.savefig(out/'neighborhood_score_distributions.png',dpi=160)
    write(out/'candidate_neighborhood_statistics.csv',rows)
    write(out/'runtime_neighborhood.csv',timing)
if __name__=='__main__':
    run_stats(Path(sys.argv[1]),Path(sys.argv[2]))
