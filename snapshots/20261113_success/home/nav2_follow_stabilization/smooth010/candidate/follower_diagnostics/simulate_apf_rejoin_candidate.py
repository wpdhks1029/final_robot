"""가상 2.5m 직선 공간, 20cm 상자, 기존 감속/회전 차체 검사. 실물 검증 아님."""
def main():
    import sys,math
    from apf_rejoin_candidate import StraightRejoin
    from apf_guard_connection import swept_clear
    from apf_case3_candidate import Settings,clearance
    c=Settings(influence=1.5,lidar_height=.25,visibility_confirmed=True);m=StraightRejoin(c,-1)
    world=[]
    for i in range(121):
     x=-1+i*.05;world.extend([(x,-1.25),(x,1.25)])
    for i in range(11):
     t=i/10;world.extend([(1.6,.25+.2*t),(1.8,.25+.2*t),(1.6+.2*t,.25),(1.6+.2*t,.45)])
    x=y=yaw=0.;last=None
    for i in range(10000):
     co,si=math.cos(yaw),math.sin(yaw);pts=[(co*(px-x)+si*(py-y),-si*(px-x)+co*(py-y)) for px,py in world]
     dx=.8;dy=-y;tx=co*dx+si*dy;ty=-si*dx+co*dy;pp_w=2*ty/(tx*tx+ty*ty)*.04
     r=m.command(.04,pp_w,pts,(x,y,yaw),[(j*.1,0.) for j in range(51)],(i+1)*100000000)
     v,w=r['v'],r['w'];d=min(clearance(px,py,c) for px,py in pts);factor=min(1.,max(0.,(d-.3)/.3));v*=factor;w*=factor
     if r['phase']!=last:print(i*.1,r['phase'],x,y,yaw,d,flush=True);last=r['phase']
     if r['reason'].startswith('STOP') or not swept_clear(pts,v,w,c):print('STOP',i*.1,r,flush=True);break
     x+=v*math.cos(yaw)*.1;y+=v*math.sin(yaw)*.1;yaw+=w*.1
     if r['phase']=='GEOMETRIC_COMPLETE_CANDIDATE':break
    else:print('END',x,y,yaw,d,flush=True)

if __name__=="__main__":main()
