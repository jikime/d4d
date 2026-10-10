import {useEffect,useRef,useState} from 'react';
import ApproachSimulator from './ApproachSimulator';
import PilotSimulator from './PilotSimulator';
import {missionFlightRoute,trainingRoute,PilotFlight} from './simulation/pilotFlight';

export interface SimulatorLaunch { route?:{x:number;y:number}[];name?:string;initialMode?:'flight'|'approach' }
export default function Simulator({onClose,launch={}}:{onClose:()=>void;launch?:SimulatorLaunch}) {
  const opener=useRef(document.activeElement as HTMLElement|null);
  const [mode,setMode]=useState(launch.initialMode??'flight');
  const mission=useRef(missionFlightRoute(launch.route??[],launch.name??'메인 임무 계획'));
  const engine=useRef<PilotFlight|null>(null);if(!engine.current)engine.current=new PilotFlight(mission.current??trainingRoute());
  useEffect(()=>()=>{if(opener.current?.isConnected)opener.current.focus();},[]);
  return mode==='flight'?<PilotSimulator engine={engine.current} mission={mission.current} missionUnavailable={Boolean(launch.route?.length&&!mission.current)} onClose={onClose} onApproach={()=>setMode('approach')}/>:<ApproachSimulator onClose={onClose} onFlight={()=>setMode('flight')}/>;
}
