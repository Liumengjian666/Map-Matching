"""Apply the Q7 research hook to the immutable Q6 continuous pipeline clone."""
from pathlib import Path
import sys

here=Path(__file__).resolve().parent
q6=here.parent/"p64_r3_q6_fine_grained_geometry_assisted_ndt"
text=(q6/"pipeline.cpp").read_text()


def replace(before,after):
    global text
    assert text.count(before)==1,before
    text=text.replace(before,after)


replace("int refine_frozen(int argc,char** argv) {",
        '#include "direction_hook.hpp"\n\nint refine_frozen(int argc,char** argv) {')
replace('mode=="LOCAL_SHADOW","bad mode"', 'mode=="LOCAL_SHADOW"||mode=="DIRECTION"||mode=="DIRECTION_DC","bad mode"')
replace("    auto target=downsample(load(map));",'''    Eigen::Vector3f axis;
    float shift=0.f;
    int forced_count=0;
    std::ifstream study(dir/"direction.tsv");
    study>>axis.x()>>axis.y()>>axis.z()>>shift>>forced_count;
    need(bool(study)&&axis.allFinite()&&std::abs(axis.norm()-1)<1e-4,"direction contract");
    seed+=axis*shift;
    const Cloud::ConstPtr world=load(map);
    auto target=downsample(world);
    pcl::KdTreeFLANN<Point> directional_tree;
    const bool directional_mode=mode=="DIRECTION"||mode=="DIRECTION_DC";
    if(directional_mode)directional_tree.setInputCloud(world);''')
replace('    auto ranking=file', '''    auto direction=file(out/"directional.csv","frame,stamp_ns,kind,accepted,retained,iterations,selection_s,fit_s,native_matrix,selected_matrix,axis_world,schur_condition,triggered");
    auto ranking=file''')
replace('          if(!local_only&&trigger!="NONE"&&mode!="B"&&searches<3){',
'''          if(directional_mode&&(mode=="DIRECTION_DC"||weak||frame<forced_count)){
            const auto q7=fit_direction(scan,world,directional_tree,searcher,base,axis,mode=="DIRECTION_DC");
            direction<<frame<<','<<ns<<','<<(mode=="DIRECTION_DC"?"DCREG_DEFAULT":frame<forced_count?"FORCED":"NATIVE_WEAK")<<','
                     <<q7.converged<<','<<q7.retained<<','<<q7.iterations<<','<<q7.selection_s<<','<<q7.fit_s<<',';
            list(direction,base);direction<<',';list(direction,q7.pose);direction<<',';list(direction,q7.axis);direction<<','<<q7.schur_condition<<','<<q7.triggered<<'\\n';
            if(q7.converged){
              update=q7.pose;
              proposal="DIRECTIONAL";
              actual=proposal;
            }
          }
          if(!directional_mode&&!local_only&&trigger!="NONE"&&mode!="B"&&searches<3){''')
Path(sys.argv[1]).write_text(text)
