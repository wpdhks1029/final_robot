"""Checked diagonal bypass: minimize initial pivot, then pass and rejoin.

All observed points and known box boundary remain collision obstacles.
A complete static route is required; live checks still gate every command.
"""
import math
import numpy as np
from apf_pivot_candidate import APFPivot, checked_step, build_apf_pivot_plan


def checked_route(points,start,box):
    # Small angle first, with whole-body rotation at every corner checked.
    # Offset reserves 34cm body clearance plus 10cm tracking allowance.
    for deg in range(20,81,5):
        heading=math.radians(deg)
        offset=max(start[1]+.2,box['high']+.74)
        length=(offset-start[1])/math.sin(heading)
        corner=(start[0]+length*math.cos(heading),offset)
        if corner[0]>=box['far']+.65:continue
        targets=[corner,(box['far']+.75,offset),(box['far']+1.65,0.)]
        pose=tuple(start);minimum=99.;path=[]
        for target in targets:
            dx,dy=target[0]-pose[0],target[1]-pose[1]
            h=math.atan2(dy,dx);distance=math.hypot(dx,dy)
            ok,d,segment=checked_step(points,pose,h,distance)
            minimum=min(minimum,d)
            if not ok or d<=.40:break
            path.extend(segment);pose=(*target,h)
        else:
            ok,d,segment=checked_step(points,pose,0.,1.1)
            if ok and d>.40:
                return dict(targets=targets,initial_heading_deg=deg,
                            min_clearance_m=min(minimum,d),path=path+segment)
    raise ValueError('STOP_NO_CHECKED_GENTLE_ROUTE')


class GentlePivot(APFPivot):
    def __init__(self,start,box,route):
        super().__init__(start,box)
        self.targets=[tuple(p) for p in route['targets']]+[(route['targets'][-1][0]+1.1,0.)]
        self.join_x=self.targets[2][0]
        self.outward_heading=math.atan2(self.targets[0][1]-start[1],self.targets[0][0]-start[0])
        self.map_rejoin=False;self.map_guidance=None
        self.side_adjustments=0
        self.allow_blocked_reselection=True

    def choose(self,pose,points):
        x,y,a=pose
        # Finish each diagonal before pivoting; do not chase a lateral waypoint
        # after its forward corridor has already been checked.
        while self.stage<(2 if self.map_rejoin else 3) and math.hypot(self.targets[self.stage][0]-x,self.targets[self.stage][1]-y)<.06:
            self.stage+=1
        if self.map_rejoin and self.stage>=2:
            if self.map_guidance is None:return None
            g=self.map_guidance
            self.stage=3 if g['path_distance_m']<=.08 and abs(g['path_heading_error_deg'])<=8 else 2
            target=g['target_local']
        elif self.stage==1:
            # Preserve the lateral room actually obtained. Re-aiming at the
            # saved height can steer back towards the box before passing it.
            self.targets[1]=(self.targets[1][0],y)
            target=self.targets[1]
        else:target=(x+.35,0.) if self.stage==3 else self.targets[self.stage]
        dx,dy=target[0]-x,target[1]-y
        h=math.atan2(dy,dx);length=min(self.step,math.hypot(dx,dy))
        remaining=math.hypot(dx,dy)
        if self.stage==0:
            # Preserve the checked initial diagonal instead of repeatedly
            # pivoting to chase the final few centimetres of a waypoint.
            h=self.outward_heading
            along=dx*math.cos(h)+dy*math.sin(h)
            cross=-dx*math.sin(h)+dy*math.cos(h)
            if along<=0 or abs(cross)>.06:
                self.last_diagnostic.update(stage=0,stage_name='MOVE_OUT',selection_failed=True,
                                            outward_cross_track_m=cross,outward_remaining_m=along)
                return None
            remaining=along;length=min(self.step,remaining)
        # Check the entire remaining leg, including the next corner rotation.
        ok,d,path=checked_step(points,pose,h,remaining)
        if not ok:
            # Recheck a small side adjustment even after returning to forward.
            # Maximum two10cm steps. Every pivot/step and the entire following
            # straight corridor must pass against all current obstacle points.
            attempts=[]
            if self.stage==1 and self.side_adjustments<2 and abs(a)<=math.radians(65):
                headings=[math.radians(deg) for deg in range(10,51,5)]
                if math.radians(25)<=a<=math.radians(65):headings.append(a)
                headings=sorted(set(headings),key=lambda h:abs(math.atan2(math.sin(h-a),math.cos(h-a))))
                for checked,adjust_heading in enumerate(headings,1):
                    end=(x+.1*math.cos(adjust_heading),y+.1*math.sin(adjust_heading))
                    forward=self.targets[1][0]-end[0]
                    valid,cd,segment=checked_step(points,pose,adjust_heading,.1)
                    valid_forward,fd,_=checked_step(points,(*end,adjust_heading),0.,max(0.,forward))
                    attempts.append(dict(heading_deg=math.degrees(adjust_heading),step_clearance_m=cd,forward_clearance_m=fd))
                    if forward>.1 and valid and valid_forward and fd>.36:
                        self.side_adjustments+=1
                        self.targets[1]=(self.targets[1][0],end[1])
                        self.last_diagnostic=dict(stage=1,stage_name='PASS_BOX',selected_heading_rad=adjust_heading,
                            apf_heading_rad=adjust_heading,checked_clearance_m=min(cd,fd),side='left',checked_candidates=checked,
                            selection_policy='CHECKED_SIDE_CLEARANCE_REPAIR',side_adjustments=self.side_adjustments,
                            remaining_leg_checked=True,forward_corridor_clearance_m=fd)
                        return adjust_heading,end,segment
            self.last_diagnostic.update(stage=self.stage,stage_name=('MOVE_OUT','PASS_BOX','REJOIN','TRACK_REFERENCE')[self.stage],
                                        selection_failed=True,blocked_remaining_leg_clearance_m=d,side_repair_attempts=attempts)
            return None
        if self.stage<3 and not (self.map_rejoin and self.stage>=1):
            next_target=self.targets[self.stage+1]
            leg_end=(x+remaining*math.cos(h),y+remaining*math.sin(h))
            nh=math.atan2(next_target[1]-leg_end[1],next_target[0]-leg_end[0])
            ok,nd,_=checked_step(points,(*leg_end,h),nh,0.)
            if not ok:return None
            d=min(d,nd)
        self.last_diagnostic=dict(stage=self.stage,stage_name=('MOVE_OUT','PASS_BOX','REJOIN','TRACK_REFERENCE')[self.stage],
            selected_heading_rad=h,apf_heading_rad=h,checked_clearance_m=d,side='left',
            checked_candidates=1,selection_policy='MINIMUM_CHECKED_INITIAL_TURN',remaining_leg_checked=True)
        return h,(x+length*math.cos(h),y+length*math.sin(h)),path


def build_gentle_plan(points,start):
    plan=build_apf_pivot_plan(points,start)
    route=checked_route(np.asarray(plan['points']),start,plan['box'])
    plan['gentle_route']=route
    plan['config'].update(selection_policy='MINIMUM_CHECKED_INITIAL_TURN',initial_heading_deg=route['initial_heading_deg'])
    plan['return_x']=route['targets'][-1][0]
    plan['path']=[[*p,0.] for p in route['path']]
    plan['geometry']=dict(reason='GENTLE_STATIC_ROUTE_CLEAR',scope='STATIC_ROUTE_REJOIN_PLUS_1M',
                          min_path_clearance_m=route['min_clearance_m'])
    return plan
