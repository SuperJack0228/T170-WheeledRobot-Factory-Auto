#!/usr/bin/env python3
"""Build reproducible offline dispatch reports. Standard library only; no robot connection.
Success means SOFTWARE reported completion, not independent verification of workpiece quality.
"""
import argparse
import collections
import csv
import html
import json
import math
from pathlib import Path


def percentile(values, p):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    x = (len(values)-1)*p
    lo = int(x)
    return values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*(x-lo)


def wilson(success, total):
    if not total:return None
    z=1.959963984540054; p=success/total; d=1+z*z/total
    mid=(p+z*z/(2*total))/d; radius=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/d
    return [max(0,mid-radius),min(1,mid+radius)]


def read_events(paths):
    events, warnings, seen = [], [], set()
    for path in paths:
        with Path(path).open(encoding='utf-8') as source:
            for number, line in enumerate(source, 1):
                try:
                    event = json.loads(line)
                    if not isinstance(event, dict) or not event.get('type'):
                        raise ValueError('missing event type')
                    # Do not double-count copied log files; schema-2 sequence is run-local.
                    key = (event.get('run_id'),event.get('seq'))
                    if all(v is not None for v in key):
                        if key in seen:
                            continue
                        seen.add(key)
                    event['_file'] = str(path)
                    event['_line'] = number
                    events.append(event)
                except (ValueError, TypeError) as error:
                    warnings.append(f'{path}:{number}: {error}')
    return events, warnings


def summarize(events, warnings=()):
    missions, runs = {}, {}
    phases, steps, failures, vision, grips = collections.defaultdict(list),collections.defaultdict(list),collections.Counter(),collections.Counter(),collections.Counter()
    for e in events:
        kind = e['type']; run=e.get('run_id','legacy'); mid=e.get('mission_id'); m=e.get('metrics',{})
        if not isinstance(m, dict):
            m={}
        if kind=='run_start':runs.setdefault(run,{})['start']=e
        if kind=='run_end':runs.setdefault(run,{})['end']=e
        if not mid:
            continue
        task=missions.setdefault((run,mid),dict(run_id=run,mission_id=mid,mission=e.get('mission',''),status='incomplete',dur_ms=None,retries=0,recoveries=0,servo_failures=0,empty_grasps=0,unknown_grasps=0,failed_grasp_attempts=0,manual_interventions=0,reported_placed=0,reported_taken=0,seen_start=False,seen_end=False))
        if kind=='mission_start':task['seen_start']=True;task['start_wall']=e.get('wall','');task['mission']=e.get('mission',task['mission'])
        if kind=='mission_end':
            task.update(seen_end=True,status=e.get('status','success' if e.get('ok') else 'failed'),dur_ms=m.get('dur_ms',e.get('dur_ms')),result=e.get('detail',e.get('result','')))
        if kind=='mission_execution':task['code']=m.get('code')
        if kind=='retry':task['retries']+=1
        if kind=='recovery':task['recoveries']+=1
        if kind=='manual_intervention':task['manual_interventions']+=1
        if kind=='part_count':
            key='reported_placed' if e.get('name')=='tray2_placed' else 'reported_taken'
            task[key]+=m.get('count',0)
        if kind=='gripper_feedback':
            grips[(e.get('name','unknown'),e.get('status','unknown'))]+=1
        if kind=='grasp_check':
            if e.get('status')=='empty_grasp':task['empty_grasps']+=1
            if e.get('status')=='unknown_gripper_feedback':task['unknown_grasps']+=1
        if kind=='grasp_attempt' and e.get('status')=='failed':task['failed_grasp_attempts']+=1
        if kind=='servo_end':
            if e.get('status')!='success':
                task['servo_failures']+=1;failures[e.get('detail','unknown')]+=1
            for key in ('circle_attempts','yolo_accepted_frames','circle_accepted_frames','tray_accepted_frames','rejected_frames','missing_frames','expired_frames'):
                vision[key]+=m.get(key,0)
        if kind in ('span','step'):
            duration=m.get('dur_ms',e.get('dur_ms'))
            if isinstance(duration,(int,float)) and math.isfinite(duration):
                (phases if kind=='span' else steps)[e.get('name',e.get('phase',e.get('step','unknown')))].append(duration)
    rows=[]
    for task in missions.values():
        task['first_pass']=task['status']=='success' and not any(task[k] for k in ('retries','recoveries','servo_failures','empty_grasps','unknown_grasps','failed_grasp_attempts','manual_interventions'))
        task['degraded_completion']=task['status']=='success' and not task['first_pass']
        rows.append(task)
    by_type={}
    for name in sorted({r['mission'] for r in rows}):
        group=[r for r in rows if r['mission']==name]
        success=sum(r['status']=='success' for r in group)
        finished=[r for r in group if r['status'] in ('success','failed')]
        by_type[name]=dict(started=len(group),success=success,failed=sum(r['status']=='failed' for r in group),incomplete=sum(r['status'] not in ('success','failed') for r in group),
                           completion_rate_all_started=success/len(group) if all(r['seen_start'] for r in group) else None,completion_rate_finished=success/len(finished) if finished else None,
                           first_pass_rate=sum(r['first_pass'] for r in group)/len(group) if all(r['seen_start'] for r in group) else None,
                           completion_wilson95_all_started=wilson(success,len(group)) if all(r['seen_start'] for r in group) else None,
                           median_ms=percentile([r['dur_ms'] for r in group if r['status']=='success'],.5),
                           p95_ms=percentile([r['dur_ms'] for r in group if r['status']=='success'],.95))
    # Closed-run duration includes idle time; do not substitute sum of concurrent phases.
    closed=[r['end'] for r in runs.values() if 'end' in r]
    observed_ms=sum(r.get('metrics',{}).get('dur_ms',0) for r in closed)
    summary=dict(missions=len(rows),orphan_missions=sum(not r['seen_start'] for r in rows),by_type=by_type,vision=dict(vision),servo_failure_reasons=dict(failures),
                 gripper_feedback_by_hand={f'{hand}/{state}':count for (hand,state),count in grips.items()},
                 runs=len(runs),closed_runs=len(closed),open_runs=len(runs)-len(closed),
                 reported_placed=sum(r['reported_placed'] for r in rows),reported_taken=sum(r['reported_taken'] for r in rows),
                 manual_interventions_recorded=sum(r['manual_interventions'] for r in rows),unattended_success_rate=None,
                 dropped_events=sum(r.get('metrics',{}).get('dropped_events',0) for r in closed),
                 writer_failures=sum(r.get('metrics',{}).get('writer_healthy',1)!=1 for r in closed),
                 parse_warnings=list(warnings),runtime_ms_closed_runs=observed_ms)
    summary['reported_placed_per_hour_closed_runs']=sum(r['reported_placed'] for r in rows if r['run_id'] in runs and 'end' in runs[r['run_id']])*3600000/observed_ms if observed_ms else None
    stats=lambda source:[dict(name=k,count=len(v),median_ms=percentile(v,.5),p95_ms=percentile(v,.95),max_ms=max(v)) for k,v in sorted(source.items())]
    return summary,rows,stats(steps),stats(phases)


def read_servo(paths):
    """Legacy CSV: session/vision diagnostics only. Never infer dispatch task success from it."""
    sessions={}
    for path in paths:
        for row in csv.DictReader(Path(path).open(encoding='utf-8')):
            sid=row.get('session')
            if not sid or sid=='0':continue
            key=(str(path),sid,row.get('hand'))
            s=sessions.setdefault(key,dict(file=str(path),servo_session=sid,hand=row.get('hand'),task_tag=row.get('task_tag',''),accepted=0,circle_accepted=0,missing=0,expired=0,reason='',observed_events=0))
            s['observed_events']+=1
            if row.get('event')=='observation' and row.get('reason')=='accepted':
                s['accepted']+=1;s['circle_accepted']+=row.get('vision_source')=='belt_circle'
            if row.get('event')=='vision' and row.get('reason')=='locked_target_missing':s['missing']+=1
            if 'expired' in row.get('reason',''):s['expired']+=1
            if row.get('phase','').lower()=='failed' or row.get('event')=='failure':s['reason']=row.get('reason','') or s['reason']
    return list(sessions.values())


def bar_chart(title, pairs, unit=''):
    pairs=list(pairs)
    if not pairs:return f'<section><h2>{html.escape(title)}</h2><p>暂无数据</p></section>'
    maximum=max((v for _,v in pairs),default=1) or 1
    height=60+len(pairs)*34
    svg=[f'<svg viewBox="0 0 900 {height}" role="img" aria-label="{html.escape(title)}">']
    for i,(name,value) in enumerate(pairs):
        y=25+i*34;w=450*value/maximum
        svg.append(f'<text x="0" y="{y+16}">{html.escape(str(name)[:42])}</text><rect x="320" y="{y}" width="{w:.2f}" height="22" rx="3" fill="#167b91"/><text x="{min(850,330+w):.2f}" y="{y+16}">{value:.2f}{html.escape(unit)}</text>')
    return f'<section><h2>{html.escape(title)}</h2>'+''.join(svg)+'</svg></section>'


def table(title,rows,fields):
    esc=lambda x:html.escape('—' if x is None else str(x))
    return '<section><h2>'+esc(title)+'</h2><div class="scroll"><table><thead><tr>'+''.join('<th>'+esc(k)+'</th>' for k in fields)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(r.get(k))+'</td>' for k in fields)+'</tr>' for r in rows)+'</tbody></table></div></section>'


def export_report(paths,out,servo=(),annotations=()):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    events,warnings=read_events(paths)
    if annotations:
        extra,errors=read_events(annotations);events+=extra;warnings+=errors
    summary,missions,steps,phases=summarize(events,warnings)
    legacy_sessions=read_servo(servo)
    structured={}
    for event in events:
        if event['type'] not in ('servo_start','servo_end'):continue
        m=event.get('metrics',{});key=(event.get('run_id'),m.get('servo_session'),m.get('hand'))
        row=structured.setdefault(key,dict(run_id=key[0],servo_session=key[1],hand=key[2],mission_id=event.get('mission_id'),task_tag=event.get('name'),source='dispatch',status='incomplete',reason='',file=''))
        if event['type']=='servo_start':row['file']=event.get('detail','')
        else:row.update(status=event.get('status'),reason=event.get('detail'),dur_ms=m.get('dur_ms'),accepted=m.get('accepted_frames',0),circle_accepted=m.get('circle_accepted_frames',0),missing=m.get('missing_frames',0),expired=m.get('expired_frames',0))
    structured_keys={(Path(r['file']).name,str(r['servo_session']),str(int(r['hand']))) for r in structured.values() if r.get('hand') is not None}
    sessions=list(structured.values())+[r for r in legacy_sessions if (Path(r['file']).name,str(r['servo_session']),str(r['hand'])) not in structured_keys]
    summary['legacy_servo_sessions']=len(legacy_sessions)
    summary['legacy_servo_failure_reasons']=dict(collections.Counter(r['reason'] for r in legacy_sessions if r['reason']))
    summary['structured_servo_sessions']=len(structured)
    for name,rows in [('missions',missions),('steps',steps),('phases',phases),('servo_sessions',sessions)]:
        fields=list(dict.fromkeys(k for r in rows for k in r))
        with (out/(name+'.csv')).open('w',encoding='utf-8-sig',newline='') as target:
            writer=csv.DictWriter(target,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    note='成功率来自程序任务结果；不等于零件实物合格率。无人成功率需完整人工干预记录，目前标为未知。未结束任务和日志中断单独列出，不能删除失败样本。并行细相位不可简单相加当作总节拍。'
    cards=''.join(f'<div><b>{html.escape(str(v))}</b><span>{html.escape(k)}</span></div>' for k,v in [('任务数',summary['missions']),('已结束/全部运行',f"{summary['closed_runs']}/{summary['runs']}"),('程序报告放置件数',summary['reported_placed']),('丢弃日志事件',summary['dropped_events']),('解析异常',len(warnings))])
    body=f'<h1>T170 自动调度试验报告</h1><p class="note">{note}</p><div class="cards">{cards}</div>'
    body+=bar_chart('任务程序完成率（以全部开始任务为分母）',[(k,v['completion_rate_all_started']*100) for k,v in summary['by_type'].items() if v['completion_rate_all_started'] is not None],'%')
    body+=bar_chart('首轮无重试完成率（不代表人工干预记录完整）',[(k,v['first_pass_rate']*100) for k,v in summary['by_type'].items() if v['first_pass_rate'] is not None],'%')
    body+=bar_chart('成功任务 P50 / P95 耗时',[(k+' '+p,v[field]/1000) for k,v in summary['by_type'].items() for p,field in [('P50','median_ms'),('P95','p95_ms')] if v[field] is not None],'s')
    body+=bar_chart('步骤 P95 耗时',[(r['name'],r['p95_ms']/1000) for r in steps],'s')
    body+=bar_chart('细相位 P95 耗时（允许并行）',[(r['name'],r['p95_ms']/1000) for r in phases],'s')
    body+=bar_chart('伺服观测来源与丢失/拒绝计数',summary['vision'].items())
    body+=bar_chart('伺服失败原因',summary['servo_failure_reasons'].items())
    body+=bar_chart('夹爪电机反馈（不是实物合格率）',summary['gripper_feedback_by_hand'].items())
    body+=bar_chart('任务失败结果',collections.Counter(r.get('result','unknown') for r in missions if r['status']=='failed').items())
    hand_outcomes=collections.Counter((('右手' if row.get('hand')==1 else '左手')+' '+str(row.get('status'))) for row in structured.values())
    body+=bar_chart('伺服执行结果（按手臂；不是夹取实物成功率）',hand_outcomes.items())
    body+=bar_chart('历史伺服终止原因（独立于调度任务结果）',summary['legacy_servo_failure_reasons'].items())
    if sessions:body+=bar_chart('历史伺服：按任务标签汇总缺失观测',collections.Counter({k:sum(r['missing'] for r in sessions if r['task_tag']==k) for k in {r['task_tag'] for r in sessions}}).items())
    body+=table('任务明细',missions,['mission_id','mission','status','dur_ms','retries','recoveries','servo_failures','empty_grasps','first_pass','reported_placed','result'])
    body+=table('历史伺服会话（不是调度任务成功率）',sessions,['servo_session','hand','task_tag','source','status','dur_ms','accepted','circle_accepted','missing','expired','reason'])
    body+='<section><h2>数据质量和统计口径</h2><pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre></section>'
    document='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>T170调度试验报告</title><style>body{font:15px system-ui,sans-serif;background:#f3f6f8;color:#19313e;max-width:1250px;margin:auto;padding:30px}section{background:white;border:1px solid #dce5ea;border-radius:12px;padding:22px;margin:20px 0}h2{font-size:18px}.note{line-height:1.8;background:#e6eff3;padding:18px}.cards{display:flex;gap:16px;flex-wrap:wrap}.cards div{background:white;padding:20px;min-width:135px}.cards b{display:block;font-size:30px}.cards span{color:#536b79}svg{width:100%;font-size:12px}.scroll{overflow:auto}table{border-collapse:collapse;font-size:12px}td,th{padding:9px;border-bottom:1px solid #ddd;text-align:left;white-space:nowrap}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style>'+body+'</html>'
    (out/'report.html').write_text(document,encoding='utf-8')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metrics',nargs='*',default=[],help='dispatch_metrics/run_*.jsonl (schema 2)')
    parser.add_argument('--servo',nargs='*',default=[],help='optional detailed servo CSVs, including historical logs')
    parser.add_argument('--annotations',nargs='*',default=[],help='optional manual-intervention JSONL; include run_id and mission_id')
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    if not args.metrics and not args.servo:parser.error('provide --metrics and/or --servo')
    result=export_report(args.metrics,args.out,args.servo,args.annotations)
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
