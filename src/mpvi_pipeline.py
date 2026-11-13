"""The minimal end-to-end MPVI pipeline for the beaker-on-hotplate task: plan -> orchestrator -> navigation and manipulation -> checker, on the simulator's world model.

    task = make_pipeline_task(plan=BEAKER_ON_HOTPLATE_PLAN)
    summary = run_episode(task, model, data, out_dir=..., record_dir=...)      # src/episode_runner.py

Planner -> postprocess() (the keyword cross-check and the tags; the primitive and the object phrases come with the planner's reply; src/mpvi_postprocess.py) -> tagged plan -> orchestrator, which grounds each
manipulation step's object phrases against the world model (a knowledge base filled from the simulator; src/world_model.py, src/grounding.py) when the step starts.

Every piece is minimal and behind its own interface (src/mpvi_modules.py, src/orchestrator.py), so each can be swapped later: the hand-written plan for the LLM planner, the
oracle checker for the VLM, the scripted manipulation for the VLA, the sim world model for perception. The episode's success is the task's own check (the beaker on the hotplate
and at rest) AND the plan finished (the arm back home with the gripper open); the episode ends with the outcome PLAN_FAILED and the failing step's reason if the orchestrator stops.
The viewer's status row shows the current step and its status as text (see src/orchestrator.py).

Contents:
    make_pipeline_task(plan, task_cfg, ...): the task dictionary for src/episode_runner.run_episode().
"""

from src.mpvi_modules import make_oracle_checker, make_scripted_manipulation, make_scripted_navigation, make_sim_world_model
from src.mock_policy import MockPolicy, make_mock_manipulation, make_mock_source
from src.grounding import make_grounder, make_operator, make_search_stub
from src.mpvi_plan import BEAKER_ON_HOTPLATE_PLAN
from src.mpvi_postprocess import postprocess
from src.mpvi_monitors import make_carry_slice_monitor, make_drop_monitor
from src.mpvi_vlm import make_vlm_checker
from src.orchestrator import make_orchestrator
from src.task_beaker_hotplate import make_task
from src.task_config import EPISODE_CFG, PICK_PLACE_CFG, SUCCESS_CFG, STAGING_CFG, TASK_CFG


def make_pipeline_task(plan=BEAKER_ON_HOTPLATE_PLAN, task_cfg=TASK_CFG, staging_cfg=STAGING_CFG, episode_cfg=EPISODE_CFG, success_cfg=SUCCESS_CFG, arm="left",
                       operator=None, search=None, stop_on_disagreement=False, checker="oracle", vlm_caller=None, drop_monitor=True, orch_cfg=None, grip_force=None, module_hook=None, instruction=None, manipulation="scripted", mock_stats_path=None,
                       run_dir=None):
    """Packages the pipeline as a task for the generic episode runner.

    Args:
        plan (dict): The plan in the paper's segment format. Defaults to the hand-written beaker-on-hotplate plan.
        task_cfg (dict): Task poses. Defaults to TASK_CFG.
        staging_cfg (dict): Staging configuration. Defaults to STAGING_CFG.
        episode_cfg (dict): Time limits; the auto timeout is raised to PICK_PLACE_CFG["episode_timeout_s"]. Defaults to EPISODE_CFG.
        success_cfg (dict): The success check's thresholds. Defaults to SUCCESS_CFG.
        arm (str): The arm that does the manipulation. Defaults to "left".
        operator (dict or None): The operator asked on an ambiguous phrase (src/grounding). Defaults to none present (the plan stops with "ambiguous reference").
        search (dict or None): The search module. Defaults to the stub ("search not implemented").
        stop_on_disagreement (bool): Stop the plan before it starts when the planner's primitive and the keywords disagree. Defaults to False.
        checker (str): "oracle" (the sim ground truth decides) or "vlm" (the VLM decides, as in the paper; the oracle is logged beside every deciding check). Defaults to "oracle".
        vlm_caller (GeminiCaller or None): The caller of the VLM checker; required when checker is "vlm". The image is rendered on demand at the tick of the gripper event (obs["render"]); no policy cameras are needed.
        drop_monitor (bool): Run the drop monitor (src/mpvi_monitors.py; it only logs). Defaults to True.
        orch_cfg (dict or None): The orchestrator's failure settings (ORCH_CFG: retry_max, knocked_over_deg). Defaults to ORCH_CFG.
        grip_force (float or None): Overrides the grip force per finger, newtons, for a test (the forced slip test); None uses the grip force table (GRIP_FORCE_TABLE is never changed).
        instruction (str or None): The typed instruction the plan was made from; shown in the viewer's status panel and the demo video (task["meta"]["panel"]). Defaults to None (not shown).
        manipulation (str): "scripted" (the scripted pick and place drive the arm, as before) or "mock" (the mock policy of src/mock_policy.py drives the arm and gripper
            as a chunk source named "vla", routed for the left arm and gripper; the scripted module still plans). Defaults to "scripted".
        mock_stats_path (str or Path or None): where the mock writes one row per query (M261). None writes nothing. Defaults to None.
        module_hook (callable or None): A test seam: module_hook(modules, model, data) -> modules, called on the module dictionary before the orchestrator is built (a test wraps a module to inject a failure).
        run_dir (str, Path or None): The run's own folder: the stance search writes its three log files there (src/stance_search_logging.py, M312). None: the search's own folder.

    Returns:
        dict: The task (see src/episode_runner.py); task["pipeline"] is filled with the orchestrator and the modules when the policy is made.
    """
    built = {}
    if manipulation not in ("scripted", "mock"):
        raise ValueError(f"manipulation is 'scripted' or 'mock', not {manipulation!r}")
    mock_box = {}                                        # M260: the mock source, made with the policy
    use_vlm = checker == "vlm"
    if checker not in ("oracle", "vlm") or (use_vlm and vlm_caller is None):
        raise ValueError("checker is 'oracle' or 'vlm' (the latter needs vlm_caller)")
    tagged = postprocess(plan, stop_on_disagreement)
    from src.stance_select import StanceSelectionError, apply_stance_selection       # the stance of the plan's first run comes from the inverse-reach region and the full check (M175, M178, M179)
    sel = apply_stance_selection(tagged, task_cfg, run_dir=run_dir)
    stance_report = sel["report"]
    if not sel["ok"]:
        raise StanceSelectionError(sel["stop"])
    task_cfg, tagged = sel["task_cfg"], sel["plan"]
    task = make_task(None, task_cfg, staging_cfg, {**episode_cfg, "timeout_after_handoff": PICK_PLACE_CFG["episode_timeout_s"]}, success_cfg)

    def make_policy(model, data):
        """Builds the modules and the orchestrator for this model and state."""
        world = make_sim_world_model(model, data, task_cfg)
        nav = make_scripted_navigation(model, data, world, task_cfg, staging_cfg)
        manip = make_scripted_manipulation(model, data, world, arm, task_cfg[f"home_q_{arm}"], task_cfg, force=grip_force)
        oracle = make_oracle_checker(model, data, world, manip["pad_pose"], arm, task_cfg, success_cfg)
        checker = make_vlm_checker(oracle, vlm_caller, arm) if use_vlm else oracle
        op = operator or make_operator()
        grounder = make_grounder(world, search or make_search_stub(), op, arm)
        monitors = ([make_drop_monitor(model, oracle, arm)] if drop_monitor else []) + [make_carry_slice_monitor(model, data, manip, arm)]       # the carry slice monitor logs the carry of every episode (M180)
        modules = {"navigation": nav, "manipulation": manip, "checker": checker, "world": world, "grounder": grounder, "monitors": monitors, "operator": op}
        if manipulation == "mock":                          # M260: the mock stands where the openpi client will sit (a chunk source; the planner is the scripted module)
            mock = MockPolicy(model, data, arm, task_cfg[f"home_q_{arm}"], force=grip_force)
            modules["manipulation"] = make_mock_manipulation(manip, mock, world, data)
            mock_box["source"] = make_mock_source(model, data, mock, world, stats_path=mock_stats_path)
        if module_hook is not None:
            modules = module_hook(modules, model, data)
        orch = make_orchestrator(tagged, modules, orch_cfg)
        built.update(monitors=monitors, orchestrator=orch, navigation=nav, manipulation=manip, checker=checker, world=world, grounder=grounder)
        return orch

    base_check = task["check_success"]

    def check_success(model, data):
        """The beaker is on the hotplate and at rest, and the plan has finished."""
        ok, diag = base_check(model, data)
        finished = bool(built and built["orchestrator"]["state"]["finished"])
        return ok and finished, {**diag, "plan_finished": finished}

    task.update(make_policy=make_policy, check_success=check_success, pipeline=built, stance_search_dir=(stance_report.get("sampler") or {}).get("stance_search_dir"))
    # the loop's settings this pipeline needs (read by experiments/demo_video.py): the sag layer's posture targets (M262), for every source
    loop_cfg = {"sag_home_q": {arm_: task_cfg[f"home_q_{arm_}"] for arm_ in ("left", "right")}}
    if manipulation == "mock":
        # the sources the loop adds after the policy, and the routes the auto mode needs
        task.update(extra_sources=lambda model, data: {"vla": mock_box["source"]},
                    loop_cfg={**loop_cfg, "auto_routes": {"arms.left": ("policy", "vla"), "gripper.left": ("policy", "vla")}})
    else:
        task.update(loop_cfg=loop_cfg)
    from src.approach_angle import choose_approach_angle, choose_place_angle       # M279: the panel shows the place's chosen approach angle (the first of its list)
    place_deg = next((float(choose_place_angle(seg["approach_angles"])) for seg in tagged["segments"] if seg.get("primitive") == "place" and seg.get("approach_angles")), None)
    first = (stance_report.get("chosen") or {})
    panel = {"instruction": instruction, "steps": [seg["description"] for seg in tagged["segments"]],      # what the status panel shows (src/viewer_layout.panel_from_status); the same text in the live window and the video
             "spot_xy": first.get("xy"), "pedestal_q": first.get("pedestal_q"), "place_theta_deg": place_deg}
    passed_try = next((t for t in stance_report["runs"][0]["full_check"] if t.get("full_check_ok")), None)      # M284: the pedestal the stance was selected at, and the height the full check passed at (the run record keeps both)
    task["meta"] = {**task["meta"], "plan": plan, "panel": panel, "sampler_record": (stance_report.get("sampler") or {}).get("record_path"),
                    "pedestal": {"selected_m": float(task_cfg["pedestal_q"]), "checked_m": None if passed_try is None else float(passed_try["q"])}}
    task["tagged_plan"] = tagged
    task["stance_selection"] = stance_report
    return task
