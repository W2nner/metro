import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
let liveMode=false,liveTimer=null,liveSequence=null,liveViewEpoch=0;
const $=id=>document.getElementById(id),view=$('viewport');
const scene=new THREE.Scene();scene.background=new THREE.Color('#050c17');
const camera=new THREE.PerspectiveCamera(60,1,.03,1500);camera.up.set(0,0,1);camera.position.set(15,12,12);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));view.appendChild(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,-25,0);controls.enableDamping=true;
scene.add(new THREE.AxesHelper(4));let cloud=null,boxes=new THREE.Group();scene.add(boxes);
let catalog,result,config,busy=false,playing=false,bag=0,frame=0,playTimer=null;
new ResizeObserver(()=>{if(!view.clientWidth||!view.clientHeight)return;camera.aspect=view.clientWidth/view.clientHeight;camera.updateProjectionMatrix();renderer.setSize(view.clientWidth,view.clientHeight)}).observe(view);
function animate(){requestAnimationFrame(animate);if($('detection-panel').hidden)return;controls.update();renderer.render(scene,camera);$('camera').textContent=`Камера XYZ: ${camera.position.toArray().map(v=>v.toFixed(2)).join(' / ')} м`}animate();
function raw(p){const r=result.basis_raw_to_forward_left_up;return new THREE.Vector3(...[0,1,2].map(i=>p.reduce((s,v,j)=>s+v*r[j][i],0)))}
function lineBox(lo,hi,color,ground=false){let vertices=[];for(let k=0;k<8;k++){let p=[0,1,2].map(j=>(k&(1<<j))?hi[j]:lo[j]);if(ground)p[2]+=result.ground_plane[0]*p[0]+result.ground_plane[1]*p[1]+result.ground_plane[2];vertices.push(raw(p))}let points=[];for(let i=0;i<8;i++)for(let j=0;j<3;j++)if(!(i&(1<<j)))points.push(vertices[i],vertices[i|(1<<j)]);let geometry=new THREE.BufferGeometry().setFromPoints(points);boxes.add(new THREE.LineSegments(geometry,new THREE.LineBasicMaterial({color,transparent:true,opacity:.8})))}
function interp(xs,ys,x){if(x<=xs[0])return ys[0];for(let i=1;i<xs.length;i++)if(x<=xs[i])return ys[i-1]+(ys[i]-ys[i-1])*(x-xs[i-1])/(xs[i]-xs[i-1]);return ys.at(-1)}
function extend(c,x,limit){let t=Math.min(x,limit);return c[0]*t*t+c[1]*t+c[2]+(x-t)*(2*c[0]*limit+c[1])}
function drawCorridor(){
 const samples=[],reliable=[],r=result.rail_model,c=result.corridor,t=result.corridor_trace,sections=result.tunnel_sections,path=result.corridor_path;
 for(let x=config.min_range_m;x<=Math.min(config.max_range_m,Math.max(40,result.max_observed_range_m));x+=3){
  let y=path?interp(path.range_m,path.lateral_m,x):config.corridor_model==='straight'?c.center_lateral_m:r?extend(r.center_coefficients,x,r.fit_max_m)+config.lateral_offset_m:t?interp(t.range_m,t.lateral_m,x):c.center_lateral_m;
  let z=path?interp(path.range_m,path.floor_m,x):r?extend(r.height_coefficients,x,r.fit_max_m):sections?.length?interp(sections.map(v=>v[0]),sections.map(v=>v[1]),x):result.ground_plane[0]*x+result.ground_plane[2];
  samples.push([-1,1].flatMap(side=>[0,c.height_m].map(h=>raw([x,y+side*c.half_width_m,z+(r?r.cross_slope*side*c.half_width_m:0)+h]))));
  reliable.push(!path||(path.rail_anchored!==false&&interp(path.range_m,path.uncertainty_lateral_m,x)<c.half_width_m&&interp(path.range_m,path.uncertainty_height_m,x)<c.height_m/2&&interp(path.range_m,path.evidence,x)>0));
 }
 const groups=[[],[]];for(let i=0;i<samples.length;i++){const points=groups[reliable[i]&&(!i||reliable[i-1])?0:1];if(i)for(let j=0;j<4;j++)points.push(samples[i-1][j],samples[i][j]);if(i%5===0)for(const [a,b] of [[0,1],[1,3],[3,2],[2,0]])points.push(samples[i][a],samples[i][b])}
 groups.forEach((points,i)=>boxes.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:i?0xedb65c:0x24d7a5,transparent:true,opacity:i ? 0.3 : 0.55}))));
}
function isAlarm(o){return o.supported&&(result.mode==='single'||o.confirmed)}
function stopPlay(){playing=false;clearTimeout(playTimer);$('play').textContent='▶'}
const warningNames={PATH_ALIGNMENT_UNVERIFIED:'Не удалось определить собственный путь — оценка неопределённа',CALIBRATION_UNVERIFIED:'Калибровка не подтверждена',SELF_MASK_NOT_CONFIGURED:'Маска собственного поезда не настроена',GROUND_ESTIMATE_UNRELIABLE:'Ненадёжная оценка пола',STRAIGHT_CORRIDOR_ASSUMPTION:'Прямой габарит',TUNNEL_RELATIVE_CORRIDOR_UNVERIFIED:'Путь оценён по геометрии облака',SPARSE_OR_OCCLUDED_BEYOND_150M:'Мало точек или перекрытие обзора за 150 м'};
async function request(url,options){const response=await fetch(url,options);if(!response.ok)throw new Error((await response.json()).error);return response}
function renderCloud(positions){
if(cloud){scene.remove(cloud);cloud.geometry.dispose();cloud.material.dispose()}
const colors=new Float32Array(positions.length);const color=new THREE.Color();for(let i=0;i<positions.length;i+=3){let d=Math.hypot(positions[i],positions[i+1],positions[i+2]);color.setHSL(.52-Math.min(d/250,1)*.4,.75,.48);colors.set([color.r,color.g,color.b],i)}
const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));cloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:.035,vertexColors:true,sizeAttenuation:true}));scene.add(cloud);
for(const obj of [...boxes.children]){boxes.remove(obj);obj.geometry.dispose();obj.material.dispose()}
drawCorridor();
for(const o of result.objects)if(isAlarm(o)||$('showUncertain').checked)lineBox(o.min,o.max,isAlarm(o)?0xff6d78:0xedb65c);
for(const o of result.nearby_objects||[])lineBox(o.min,o.max,0x70b6ff);
$('status').textContent={OBSTACLE:'Препятствие в габарите',UNCERTAIN:'Недостаточно подтверждённых данных',NO_OBSTACLE_DETECTED:'Тревог не обнаружено'}[result.status];
$('status').style.color=result.obstacle?'#ff7d87':result.status==='UNCERTAIN'?'#eac17c':'#69dfb7';
$('distance').textContent=result.distance_m?.toFixed(1)??'—';$('points').textContent=result.valid_points.toLocaleString('ru');$('latency').textContent=result.processing_ms.toFixed(1);$('far').textContent=result.points_ge_150m;$('maxrange').textContent=result.max_observed_range_m.toFixed(0)+' м';
$('warnings').textContent=result.warnings.map(w=>warningNames[w]||w).join(' · ');
const path=result.corridor_path;
if(path){const end=Math.min(100,result.max_observed_range_m),a=interp(path.range_m,path.lateral_m,10),b=interp(path.range_m,path.lateral_m,30),y=interp(path.range_m,path.lateral_m,end),bend=y-(a+(b-a)*(end-10)/20);$('route').textContent=`Оценка пути до ${end.toFixed(0)} м: ${Math.abs(bend)<.5?'близок к прямому':bend>0?'изгиб влево':'изгиб вправо'}. Зелёный габарит используется детектором.`}else $('route').textContent='Путь: заданный прямой габарит';
$('objects').replaceChildren();
const visible=[...result.objects.filter(isAlarm).map(o=>({...o,caption:'ТРЕВОГА'})),...(result.nearby_objects||[]).map(o=>({...o,caption:'Рядом с путём'}))];
for(const o of visible){const row=document.createElement('button');row.className='object';row.textContent=`${o.distance_m.toFixed(2)} м`;const detail=document.createElement('span');detail.textContent=o.caption;row.appendChild(detail);row.onclick=()=>{const center=raw(o.center);controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(5,7,4))};$('objects').appendChild(row)}
const uncertain=result.objects.filter(o=>!isAlarm(o)).length;
$('counts').textContent=`Тревог: ${result.objects.filter(isAlarm).length} · рядом: ${(result.nearby_objects||[]).length} · неопределённых: ${uncertain}`;
if(!visible.length)$('objects').textContent='Тревог и объектов рядом нет';
$('frame').value=frame;$('timeline').value=frame;
}
async function load(){if(busy||liveMode)return;busy=true;$('busy').hidden=false;for(const id of ['dataset','timeline','frame','apply','prev','next'])$(id).disabled=true;
try{const query=`?bag=${bag}&frame=${frame}`;result=await(await request('/api/frame'+query)).json();const positions=new Float32Array(await(await request('/api/cloud'+query)).arrayBuffer());renderCloud(positions);
}catch(e){playing=false;$('warnings').textContent='Ошибка: '+e.message}finally{busy=false;$('busy').hidden=true;for(const id of ['dataset','timeline','frame','apply','prev','next'])$(id).disabled=false;if(playing)playTimer=setTimeout(next,100)}}
function next(){if(busy)return;if(frame>=catalog.datasets[bag].frames-1){playing=false;$('play').textContent='▶';return}frame++;load()}
$('dataset').onchange=()=>{stopLiveView();stopPlay();if(!catalog.datasets.length)return;bag=Number($('dataset').value);frame=0;$('timeline').max=$('frame').max=catalog.datasets[bag].frames-1;$('total').textContent='/ '+(catalog.datasets[bag].frames-1);load()};
$('timeline').onchange=()=>{stopPlay();frame=Number($('timeline').value);load()};$('frame').onchange=()=>{stopPlay();frame=Math.max(0,Math.min(catalog.datasets[bag].frames-1,Number($('frame').value)||0));load()};$('prev').onclick=()=>{stopPlay();frame=Math.max(0,frame-1);load()};$('next').onclick=()=>{stopPlay();next()};$('showUncertain').onchange=()=>{stopPlay();load()};$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'Ⅱ':'▶';if(playing&&!busy)next()};
$('apply').onclick=async()=>{stopPlay();try{config=await(await request('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:$('mode').value,margin_enabled:$('margin').checked,margin_lateral_m:Number($('marginSide').value),margin_top_m:Number($('marginTop').value),lateral_offset_m:Number($('offset').value),yaw_deg:Number($('yaw').value)})})).json();await load()}catch(e){$('warnings').textContent=e.message}};
function download(data,name){const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}$('export').onclick=()=>download(result,`detection-${bag}-${frame}.json`);$('saveConfig').onclick=()=>download(config,'detector-config.json');
$('overview').onclick=()=>{camera.position.set(15,12,12);controls.target.set(0,-25,0)};$('origin').onclick=()=>{camera.position.set(0,0,0);controls.target.copy(raw([30,0,0]));controls.update()};$('top').onclick=()=>{const span=Math.min(180,Math.max(60,result?.max_observed_range_m||100));camera.position.copy(raw([span/2-.01,0,span*1.25]));controls.target.copy(raw([span/2,0,0]));controls.update()};
renderer.domElement.addEventListener('click',event=>{if(!event.shiftKey||!cloud)return;const r=renderer.domElement.getBoundingClientRect(),mouse=new THREE.Vector2((event.clientX-r.left)/r.width*2-1,-(event.clientY-r.top)/r.height*2+1);const ray=new THREE.Raycaster();ray.params.Points.threshold=.12;ray.setFromCamera(mouse,camera);const hit=ray.intersectObject(cloud)[0];if(hit){const p=new THREE.Vector3().fromBufferAttribute(cloud.geometry.attributes.position,hit.index);$('picked').textContent=`Точка XYZ: ${p.toArray().map(v=>v.toFixed(3)).join(' / ')} м · дальность ${p.length().toFixed(3)} м`}});
try{await refreshCatalog();$('mode').value=config.mode;$('margin').checked=config.margin_enabled;$('marginSide').value=config.margin_lateral_m;$('marginTop').value=config.margin_top_m;$('offset').value=config.lateral_offset_m;$('yaw').value=config.yaw_deg;if(catalog.datasets.length)$('dataset').onchange();else{$('busy').hidden=true;$('warnings').textContent='Загрузите облако или подключите поток';}}catch(e){$('busy').textContent=e.message}

async function refreshCatalog(selected){catalog=await(await request('/api/catalog')).json();config=catalog.config;if(catalog.initial_bag&&!$('sourcePath').value)$('sourcePath').value=catalog.initial_bag;$('dataset').replaceChildren();for(const b of catalog.datasets){const o=document.createElement('option');o.value=b.id;o.textContent=b.name;$('dataset').appendChild(o)}if(selected!==undefined)$('dataset').value=selected;$('mountHint').textContent=catalog.data_root?'Доступная папка: '+catalog.data_root:'Папка данных не подключена. Загрузите облако кнопкой или перезапустите с --data.';}
function stopLiveView(){liveViewEpoch++;liveMode=false;clearTimeout(liveTimer);$('dataset').disabled=false;for(const id of ['timeline','frame','prev','next','play'])$(id).disabled=false;}
async function pollLive(epoch=liveViewEpoch){if(!liveMode||epoch!==liveViewEpoch)return;try{const packet=await(await request('/api/live/frame'+(liveSequence===null?'':'?after='+liveSequence))).json();if(epoch!==liveViewEpoch)return;const st=packet.stream;$('sourceStatus').textContent=`Поток: ${st.kind} · принято ${st.received} · обработано ${st.processed} · пропущено ${st.dropped}`;if(st.error)$('sourceStatus').textContent+=' · Ошибка: '+st.error;if(!packet.waiting&&packet.sequence!==liveSequence){liveSequence=packet.sequence;result=packet.result;const raw=Uint8Array.from(atob(packet.cloud_base64),c=>c.charCodeAt(0));renderCloud(new Float32Array(raw.buffer));$('busy').hidden=true;}if(packet.waiting)$('busy').textContent=st.error||'Ожидание PointCloud2…';}catch(e){$('warnings').textContent=e.message;}finally{if(liveMode&&epoch===liveViewEpoch)liveTimer=setTimeout(()=>pollLive(epoch),150)}}
window.addEventListener('metro-source',async e=>{if(e.detail.live){stopPlay();stopLiveView();liveMode=true;liveSequence=null;$('busy').hidden=false;for(const id of ['dataset','timeline','frame','prev','next','play'])$(id).disabled=true;pollLive();}else{stopLiveView();await refreshCatalog(e.detail.dataset);$('dataset').onchange();}});
window.addEventListener('metro-stop',()=>{stopLiveView();$('busy').hidden=true;$('sourceStatus').textContent='Поток остановлен';if(catalog.datasets.length)load();});
window.addEventListener('metro-open-frame',async e=>{stopLiveView();stopPlay();await refreshCatalog(e.detail.dataset);bag=e.detail.dataset;frame=e.detail.frame;$('timeline').max=$('frame').max=catalog.datasets[bag].frames-1;$('total').textContent='/ '+(catalog.datasets[bag].frames-1);load();});
window.addEventListener('metro-config',async()=>{await refreshCatalog(bag);$('mode').value=config.mode;$('margin').checked=config.margin_enabled;$('marginSide').value=config.margin_lateral_m;$('marginTop').value=config.margin_top_m;$('offset').value=config.lateral_offset_m;$('yaw').value=config.yaw_deg;if(!liveMode&&catalog.datasets.length)load();});
