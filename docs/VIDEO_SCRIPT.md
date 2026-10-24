# Demo video script (target 4:40, hard limit 5:00)

Record the screen at 1080p. Before recording:

- Start the demo once, so the URL stays the same through all takes:
  `ssh ubuntu@<ec2> 'cd ~/OpenCVH && setsid bash scripts/serve_demo.sh ~/runs/euroc3_r025_d 2 < /dev/null'`
- Restart the replay right before shot 4 (`serve_demo.sh restart …`) and click
  **Enable desktop alerts + sound** in the dashboard.
- Have these open in tabs: the dashboard, `docs/figures/architecture.png`,
  `docs/figures/merge_before_after.png`, and `docs/REPORT.md` on GitHub.

Narration is written to be read at a calm pace (about 140 words/min).

---

## 1. Team and problem (0:00–0:30)

**Screen:** team on camera, or a title card ("Swarm coverage", names, OpenCV 5 hackathon, Agentic Vision).

> We're [team names]. When several people map a building with cameras, they film the same rooms twice,
> they don't notice when their footage stops being usable, and a map built from wrong matches fails
> silently. We built a system where three cameras map together, and an agent tells each walker what to
> do next, but asks a human whenever the visual evidence is weak.

## 2. Architecture (0:30–1:10)

**Screen:** `architecture.png`; move the cursor along the flow.

> Every camera's stereo pair first goes through our OpenCV 5 front end. It rectifies the pair and scores
> it: blur, ORB features, and two geometric checks. Swarm-SLAM, running three robots on one EC2 instance,
> merges the maps. A recorder turns all of that into events. The coordinator, Amazon Nova on Bedrock
> with tool calls, decides what to tell each walker. Before trusting any overlap it re-checks the two
> images with OpenCV 5, and the guardrail that blocks weak overlaps lives in code, not in the prompt.
> We didn't have cameras, so the three walkers are the EuRoC Machine Hall flights: real stereo footage
> with laser ground truth.

## 3. OpenCV 5 doing real work (1:10–1:35)

**Screen:** the dashboard's three live camera tiles; point at the numbers under each one.

> These are the rectified frames from all three cameras, live. On raw EuRoC pairs, zero percent of
> left-right matches line up on the same row; after our OpenCV 5 rectification it's ninety percent.
> Under each camera you see its blur and tracking score next to the thresholds we tuned on recorded
> runs.

## 4. The agent acting (1:35–2:50)

**Screen:** the dashboard, replay running. Wait for an "Area covered" message, then for the amber banner.

> Here two cameras are looking at the same machinery. Swarm-SLAM proposes an overlap, OpenCV 5 finds
> 225 consistent matches spread across both images, so the agent marks the area covered and tells
> camera 0, "area covered by camera 1, move on."

*(banner appears, beep)*

> Now a weaker one: only 26 matches. The agent doesn't guess. Coverage is put on hold and I'm asked.
> The card shows both keyframes with the matches drawn. Same netting, same pallet, so I approve. The
> coordinator applies it, and the log credits the decision to a human.

**Screen:** click **Review now**, then **Approve**, then scroll to the decision log.

> Every row here records the OpenCV and Swarm-SLAM numbers that triggered the action, who decided, and
> how long it took.

## 5. Results (2:50–3:40)

**Screen:** `merge_before_after.png`, then the results table in the README.

> One alignment places all three robots on the laser ground truth within 8 to 9 centimetres. On the
> left you can see a bug we found along the way: a frame mismatch that turned one robot 120 degrees. On
> the right is the fixed, merged map. The hardest flight improves from 24 centimetres alone to 7 merged.
> Every "area covered" claim we could check against ground truth was correct. Recall is lower, because
> Swarm-SLAM only reports overlaps every few seconds, and we show that on the map instead of hiding it.

## 6. Failure cases (3:40–4:25)

**Screen:** `docs/figures/perturb_blank.jpg` and `perturb_shake.jpg`, then the failure table in REPORT.md.

> We injected the spec's failures. A blank wall and a quick shake trigger "slow down" within a third of
> a second. The first time, SLAM never recovered after the wall, and our agent kept saying "tracking
> OK", because the images looked fine again. We fixed both: odometry now resets, and SLAM is back 1.6
> seconds after the wall, and the agent now warns "tracking lost, walk back" whenever good frames
> arrive but no map keyframes do. For look-alike places, we planted six false matches; four went to a
> human, two still slipped through, and we report that.

## 7. Close (4:25–4:40)

**Screen:** the GitHub repo front page.

> Everything is measured, logged and reproducible from the repository: the OpenCV 5 front end, the
> agent, the dashboard and the evaluation. Thanks for watching.

---

**Checklist after recording:** under 5:00 · team shown · app working (shots 3–4) · architecture (shot 2)
· main results (shots 5–6) · upload as public or unlisted · put the link in the submission.
