import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
let closeCurrent=null;
export async function openInspection(id){
 closeCurrent?.();
 const dialog=document.createElement('dialog');dialog.className='inspection';
 dialog.innerHTML=`<header><div><span class="eyebrow">СОХРАНЁННОЕ ОБНАРУЖЕНИЕ</span><h2>Загрузка облака…</h2></div><button aria-label="Закрыть просмотр">✕ Закрыть</button></header><div class="inspection-layout"><div class="inspection-stage"><div class="inspection-canvas"></div><div class="inspection-tools"><button data-view="object" class="primary">К препятствию</button><button data-view="all">Всё облако</button></div><p class="inspection-legend">Красный — тревога · голубой — облако · мышь: вращение и масштаб</p></div><aside><p class="inspection-status" role="status">Чтение сохранённого результата…</p><div class="inspection-facts"></div><h3>Объекты в кадре</h3><div class="inspection-objects"></div><p class="sub">Показан результат на момент записи. Просмотр не запускает детекцию повторно.</p></aside></div>`;
 document.body.append(dialog);dialog.showModal();
 const controller=new AbortController();let disposed=false,renderer,controls,observer,animation,scene;
 const close=()=>{if(disposed)return;disposed=true;controller.abort();cancelAnimationFrame(animation);observer?.disconnect();controls?.dispose();scene?.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});renderer?.dispose();dialog.close();dialog.remove();if(closeCurrent===close)closeCurrent=null;};closeCurrent=close;
 dialog.querySelector('header button').onclick=close;dialog.addEventListener('cancel',e=>{e.preventDefault();close();});
 try{
  const response=await fetch('/api/history/item?id='+encodeURIComponent(id),{signal:controller.signal});const item=await response.json();if(!response.ok)throw Error(item.error);
  if(disposed)return;
  dialog.querySelector('h2').textContent=item.source+' · '+(item.frame===null?'поток':'кадр '+item.frame);
  const status=dialog.querySelector('.inspection-status'),facts=dialog.querySelector('.inspection-facts');
  const verdict={unreviewed:'Не проверено',true:'Отмечено верным',false:'Отмечено ложным'}[item.verdict];
  for(const text of [`${item.distance.toFixed(2)} м от лидара`,verdict,`Режим: ${item.mode==='confirmed'?'с подтверждением':'одно облако'}`]){const p=document.createElement('p');p.textContent=text;facts.append(p);}
  const host=dialog.querySelector('.inspection-canvas');scene=new THREE.Scene();scene.background=new THREE.Color('#07121e');
  const camera=new THREE.PerspectiveCamera(50,1,.03,3000);camera.up.set(0,0,1);
  renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(renderer.domElement);
  controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
  const basis=item.result.basis_raw_to_forward_left_up;
  const raw=p=>new THREE.Vector3(...[0,1,2].map(i=>p.reduce((s,v,j)=>s+v*basis[j][i],0)));
  const objects=item.result.objects.filter(o=>o.supported&&(item.mode==='single'||o.confirmed));
  let selected=objects[0],cloudBox=null;
  function focus(){if(!selected)return;const center=raw(selected.center),span=Math.max(2,...selected.dimensions)*2.2;controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(span,span*.8,span*.65));controls.update();}
  const objectButtons=[];
  objects.forEach((object,index)=>{
   const corners=[];for(const x of [object.min[0],object.max[0]])for(const y of [object.min[1],object.max[1]])for(const z of [object.min[2],object.max[2]])corners.push(raw([x,y,z]));
   const edges=[];for(let a=0;a<8;a++)for(const bit of [1,2,4])if((a^bit)>a)edges.push(corners[a],corners[a^bit]);
   scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(edges),new THREE.LineBasicMaterial({color:0xff4f66,depthTest:false})));
   const button=document.createElement('button');button.className='inspection-object';button.textContent=`${index+1}. Объект · ${object.distance_m.toFixed(2)} м\n${object.dimensions.map(v=>v.toFixed(2)).join(' × ')} м`;
   button.onclick=()=>{selected=object;objectButtons.forEach(b=>b.classList.toggle('active',b===button));focus();};objectButtons.push(button);dialog.querySelector('.inspection-objects').append(button);
  });objectButtons[0]?.classList.add('active');
  dialog.querySelector('[data-view="object"]').onclick=focus;
  dialog.querySelector('[data-view="all"]').onclick=()=>{if(!cloudBox||cloudBox.isEmpty())return;const c=cloudBox.getCenter(new THREE.Vector3()),span=Math.max(3,cloudBox.getSize(new THREE.Vector3()).length());controls.target.copy(c);camera.position.copy(c).add(new THREE.Vector3(span*.6,span*.6,span*.5));controls.update();};
  observer=new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;if(w&&h){renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}});observer.observe(host);
  function animate(){if(disposed)return;animation=requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);}focus();animate();
  if(!item.cloud_available){status.textContent='Облако этой старой записи не сохранено. Показаны только границы объекта.';return;}
  const cloud=await fetch('/api/history/cloud?id='+encodeURIComponent(id),{signal:controller.signal});if(!cloud.ok)throw Error('Не удалось прочитать облако');const bytes=await cloud.arrayBuffer();if(disposed)return;
  const points=new Float32Array(bytes),geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(points,3));geometry.computeBoundingBox();cloudBox=geometry.boundingBox;
  // Highlight original returns inside saved cuboids; no synthetic points are added.
  const colors=new Float32Array(points.length);
  for(let n=0;n<points.length;n+=3){const p=[0,1,2].map(j=>basis[j][0]*points[n]+basis[j][1]*points[n+1]+basis[j][2]*points[n+2]);const hit=objects.some(o=>p.every((v,j)=>v>=o.min[j]-.03&&v<=o.max[j]+.03));colors.set(hit?[1,.23,.32]:[.27,.72,.82],n);}
  geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));scene.add(new THREE.Points(geometry,new THREE.PointsMaterial({size:2.5,sizeAttenuation:false,vertexColors:true})));
  status.textContent=`${(points.length/3).toLocaleString('ru')} точек · ${item.cloud_kind==='snapshot'?'сохранённый обзор потока':'исходное облако'} (до 150 000 точек для просмотра)`;
 }catch(error){if(!disposed)dialog.querySelector('.inspection-status').textContent='Ошибка: '+error.message;}
}
