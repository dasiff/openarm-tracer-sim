# MPVI paper excerpts (verbatim)
Source: "Make Your VLA More Robust Without More Data By Interleaving Motion Planning", arXiv:2606.00985v1, 31 May 2026. Extracted by Claude from the PDF David provided, 2026-10-20. Short quotes for design reference only.

E1. Section 4, Completion Checker:
"Reliable detection of manipulation subtask completion is essential for triggering handoff back to the motion planner. Periodic VLM polling is unsuitable, as queries are computationally expensive and each query introduces an independent risk of hallucination, whereby a single false positive prematurely terminates manipulation and induces cascading failures in subsequent subtasks. We therefore condition VLM queries on proprioceptive signals, triggering a completion check only when gripper or arm state indicates that a manipulation primitive such as a pick or place has just terminated. Our implementation uses Gemini 2.5 Flash [31] as the completion checker."

E2. Figure 3b caption:
"(Left, red) A fixed-period query at frame 900 passes all three camera views to the VLM. The trash can is visible in the head view, so the VLM returns a false positive judgment despite the gripper not having closed; this false positive cascades into subsequent subtasks. (Right, blue) Our proprioceptive trigger fires at frame 1020, once the right gripper closes and holds at a nonzero width, and directs the VLM to the corresponding wrist view (R WRIST), yielding reliable completion detection (gain quantified in Sec. 5.3)."

E3. Appendix C.1, post-processing:
"A post-processing step extracts additional metadata used for subtask completion detection. First, we check whether each manipulation subtask can be mapped to a manipulation primitive (e.g., pick or place) with a pre-defined completion heuristic. Second, we label manipulation subtasks that immediately precede navigation subtasks, since detecting completion of these subtasks is necessary to trigger the transition to the motion planner. This information is consumed by the completion checker (Appx. C.4)."

E4. Appendix C.2, grouping:
"Given the plan P, the orchestrator groups segments into execution units: navigation segments are executed individually, while consecutive manipulation segments are grouped into a single execution unit. For each navigation segment si the orchestrator invokes the navigation module with target ti. For each manipulation group, the orchestrator invokes the manipulation module with the concatenated subtask descriptions {δj, . . . , δk} and the completion criterion ϕk of the final segment in the group, where j and k are the indices of the first and last manipulation segments in the group, respectively."

E5. Appendix C.2, failure:
"The orchestrator additionally tracks subtask completion status; if a module fails to complete its subtask, the orchestrator marks the subtask as failed and proceeds to the next segment in the plan."

E6. Appendix C.4, Completion Checker:
"A VLM-based completion checker receives the head-camera image and the completion criterion ϕ and returns a binary completion judgment; the completion-checking prompt is in Appx. D."
"Subtask sequencing. While the planner generates granular manipulation subtasks, completion is verified only for the terminal state preceding a navigation subtask. For instance, a plan may contain sequential subtasks "pick up the soda can" followed by "place the soda can in the trash can," but the VLM is queried only to verify that "there is one soda can in the trash can" prior to transitioning to navigation."
"Proprioceptive trigger. Proprioceptive signals, rather than periodic polling, trigger VLM completion checks. A set of general proprioceptive heuristics monitor gripper and arm state for all manipulation subtasks, with additional heuristics applied when a manipulation primitive has been identified during post-processing. For example, a pick primitive initiates a completion check when gripper and arm state indicate a successful grasp, while a place primitive initiates a check upon object release. This ensures that VLM queries occur at semantically meaningful moments and reduces unnecessary calls. If the VLM does not confirm completion, the VLA continues execution until a timeout is reached."

(Added 2026-10-21; extracted by Claude from the PDF David provided.)

E7. Section 4, Navigation:
"We therefore use A* motion planning for all base motion, using the VLA only inside the manipulation handoff radius around the target."

E8. Appendix C.3:
"Navigation terminates when the robot is within the handoff distance dh of t, at which point control transitions to the VLA for manipulation. Robot base repositioning within the handoff distance, which is not a task or object specific parameter, is handled by the VLA."

E9. Section 5, limitations:
"We attribute regression in task success to excessive motion planner intervention in scenarios where the VLA exhibits superior performance, such as maneuvering through narrow doorways while carrying objects."
