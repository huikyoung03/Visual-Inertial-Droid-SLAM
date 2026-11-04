"""Read-only viewer endpoints for SLAM/main.py's existing run_slam_job pipeline.

Does not execute DROID-SLAM. Uses slam_job.json from the existing server.
"""
import json
import re
from pathlib import Path
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, HTMLResponse

router = APIRouter()
SESSION_RE = re.compile(r"^[A-Za-z0-9_-]{1,100}$")

def _session(session_id):
    if not SESSION_RE.fullmatch(session_id):
        raise HTTPException(400, "Invalid session ID")
    # This module is installed next to main.py in /home/ubuntu/SLAM.
    base = Path(__file__).resolve().parent / "uploads"
    path = (base / session_id).resolve()
    if not path.is_relative_to(base.resolve()) or not path.is_dir():
        raise HTTPException(404, "Session not found")
    return path

def _job(session):
    p = session / "slam_job.json"
    if not p.is_file():
        return {"status":"not_started","message":"DROID-SLAM 작업이 아직 시작되지 않았습니다."}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"status":"failed","error":"slam_job.json cannot be read"}

def _ply_file(session, job):
    raw = job.get("ply")
    if not raw:
        return None
    ply = Path(raw).resolve()
    if session.resolve() not in ply.parents or ply.suffix.lower() != ".ply":
        return None
    if not ply.is_file() or ply.stat().st_size < 50:
        return None
    return ply

@router.get("/session/{session_id}/map-status")
def map_status(session_id: str):
    session = _session(session_id)
    job = _job(session)
    status = job.get("status", "not_started")
    if status == "done":
        if _ply_file(session, job):
            return {"state":"ready", "message":"PLY 지도 생성 완료", "mode":job.get("mode"), "ply_url":f"/session/{session_id}/map.ply"}
        return {"state":"error", "message":"작업은 완료되었지만 PLY 파일이 없습니다."}
    if status == "failed":
        return {"state":"error", "message":str(job.get("error") or "SLAM processing failed")}
    if status in ("queued", "running"):
        return {"state":"processing", "message":f"{job.get('mode', 'SLAM')} 처리 중", "mode":job.get("mode")}
    return {"state":"not_started", "message":"촬영을 중지하면 SLAM 처리가 시작됩니다."}

@router.get("/session/{session_id}/map.ply")
def get_map(session_id: str):
    session = _session(session_id)
    ply = _ply_file(session, _job(session))
    if ply is None:
        raise HTTPException(404, "PLY not ready")
    return FileResponse(
        ply,
        media_type="application/octet-stream",
        filename=f"{session_id}.ply",
        content_disposition_type="inline",
        headers={"Cache-Control": "no-store, max-age=0"},
    )

@router.get('/session/{session_id}/viewer', response_class=HTMLResponse)
def viewer(session_id: str):
    _session(session_id)  # validate
    # same-origin URLs, no cross-origin network needed except Three.js CDN
    sid = json.dumps(session_id)
    return HTMLResponse('''<!doctype html><html lang="ko"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>DROID-SLAM 3D 지도</title><style>
html,body{margin:0;width:100%;height:100%;background:#0b1220;color:#fff;font-family:sans-serif}
#canvas{position:absolute;inset:0} #info{position:absolute;z-index:3;left:12px;right:12px;top:12px;
background:#1e293bcf;padding:12px;border-radius:10px;font-size:14px}button{padding:9px;margin-top:7px}
</style></head><body><div id="canvas"></div><div id="info">지도를 확인하는 중...</div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.1/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.1/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {TrackballControls} from 'three/addons/controls/TrackballControls.js';
import {PLYLoader} from 'three/addons/loaders/PLYLoader.js';
const sid = __SESSION_ID__;
const base = '/session/' + encodeURIComponent(sid);
const info = document.getElementById('info');
const root = document.getElementById('canvas');
let loaded = false;
async function check(){
 if(loaded) return;
 try {
 const r = await fetch(base+'/map-status',{cache:'no-store'});
 if(!r.ok) throw Error('서버 상태 요청 실패: '+r.status);
 const s = await r.json();
 if(s.state==='ready'){loaded=true;info.textContent='PLY 불러오는 중...';showMap(base+'/map.ply?rev='+Date.now());return;}
 if(s.state==='error'){info.textContent='처리 실패: '+s.message;return;}
 info.textContent=s.state==='processing'?'DROID-SLAM 처리 중... 잠시 뒤 자동으로 지도가 표시됩니다.':s.message;
 }catch(e){info.textContent=String(e)}
 setTimeout(check,2500);
}
function showMap(url){
 const scene=new THREE.Scene(); scene.background=new THREE.Color(0x0b1220);
 const camera=new THREE.PerspectiveCamera(60,innerWidth/innerHeight,0.001,100000);
 const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setSize(innerWidth,innerHeight);
 renderer.setPixelRatio(Math.min(devicePixelRatio,2));root.appendChild(renderer.domElement);
 const controls=new TrackballControls(camera,renderer.domElement);
 controls.rotateSpeed=3.0;controls.zoomSpeed=1.2;controls.panSpeed=0.8;
 controls.dynamicDampingFactor=0.12;
 const loader=new PLYLoader();
 loader.load(url,(geo)=>{
 geo.computeBoundingBox(); const bounds=geo.boundingBox;
 if(!bounds || bounds.isEmpty()){info.textContent='PLY에 표시할 점이 없습니다.';return;}
 const center=bounds.getCenter(new THREE.Vector3());const size=bounds.getSize(new THREE.Vector3());
 geo.translate(-center.x,-center.y,-center.z);
 const extent=Math.max(size.x,size.y,size.z,0.1);
 const mat=new THREE.PointsMaterial({size:Math.max(extent/600,0.002),vertexColors:geo.hasAttribute('color'),color:0xe1e7f0,sizeAttenuation:true});
 scene.add(new THREE.Points(geo,mat));camera.position.set(extent*1.25,extent*.7,extent*1.25);
 camera.near=Math.max(extent/10000,0.0001);camera.far=extent*1000;camera.updateProjectionMatrix();
 controls.target.set(0,0,0);controls.update();
 info.innerHTML='3D 지도 · 모든 축 자유 회전 / 두 손가락 확대·이동<br><a style="color:#93c5fd" href="'+url+'">PLY 파일 열기</a>';
 },undefined,(e)=>{info.textContent='PLY 로딩 오류: '+(e.message||e)});
 addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight)});
 function tick(){requestAnimationFrame(tick);controls.update();renderer.render(scene,camera)}tick();
}
check();
</script></body></html>'''.replace('__SESSION_ID__',sid), headers={"Cache-Control": "no-store, max-age=0"})
