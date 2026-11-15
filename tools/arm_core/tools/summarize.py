from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parents[1]
def main():
    reports={p.stem:json.loads(p.read_text(encoding='utf-8')) for p in (ROOT/'reports').glob('*.json') if p.stem not in ['summary','source_manifest']}
    closed=reports['closed_loop']
    summary={'release':'v0.2-simulation','date':'2026-11-15','core_language':'C11 float, no heap allocation',
      'numerical_cases':reports['numerical']['samples'],'near_seed_IK_success':reports['numerical']['ik_near_seed_success'],
      'closed_loop_cases':len(closed),'worst_final_position_error_mm':max(x['position_error_m'] for x in closed)*1000,
      'worst_final_orientation_error_deg':max(x['orientation_error_rad'] for x in closed)*180/3.141592653589793,
      'max_settled_time_s':max(x['settled_time_s'] for x in closed),
      'regression_tests_passed':all(x['passed'] for x in closed) and reports['numerical']['passed'] and reports['dh']['passed'] and reports['component_dynamics']['passed'] and all(reports['native'][key]=='PASS' for key in ['core_tests','driver_overlay_tests','application_workflow_tests','application_startup_timeout_test']) and reports['infantry_compile']['adapter_and_driver_overlay_compiled'],
      'workspace_survey':reports['workspace']['survey_counts'],'workspace_cases':reports['workspace']['case_counts'],
      'workspace_executed_cases_all_passed':reports['workspace']['all_executed_cases_passed'],
      'workspace_IK':reports['workspace']['IK_counts'],
      'hardware_ready':False,'full_Infantry_linked':False,'hardware_WCET_measured':False,'global_workspace_validated':False}
    (ROOT/'reports/summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    manifest={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['src','include','config','integration','sim','python','tests','tools'] for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    for relative in ['README.md','CMakeLists.txt','build/libarm_core.so','build/libarm_dm_bridge.so','build/cortex_m4/libarm_core.a','../arm_description/heavy_arm.urdf','../arm_description/heavy_arm.xml','../arm_description/model_config.json','../arm_description/model_definition.json','../arm_description/tools/cad_reference/assembly_mates.json']:
        manifest[relative]=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
    (ROOT/'reports/source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
