"""Bedrock (Amazon Nova) coordinator with tool calls.

The model decides; the tools act. Guardrails live in the tools, not the prompt:
mark_covered refuses a low-confidence overlap until a human has approved it, so the
coverage record cannot change without the approval even if the model tries.
Every tool action is written to the decision log with the values that triggered it.
"""
import boto3
from botocore.config import Config

SYSTEM = """You coordinate three people carrying stereo cameras through a building to map it.
You receive one event at a time: an inter-robot overlap that Swarm-SLAM detected, a
tracking-quality drop measured by the OpenCV 5 front end, or Swarm-SLAM tracking being lost or
recovered for a camera. Use the tools to act, then stop.

Rules:
- Overlap event: call verify_overlap first. If confidence is "high", call mark_covered and
  notify the camera in arrived_second (NOT necessarily the higher number) with the message
  in if_covered_notify.
  If confidence is "low", call request_approval with a short reason; do NOT mark it covered.
- Quality event: notify that camera with exactly "Camera N, slow down, tracking is weak."
  using its number, unless get_quality shows it has already recovered.
- Tracking-lost event: notify that camera with exactly
  "Camera N, tracking lost: walk back to your last mapped area (PLACE) and hold still."
  using its number and the place given in the event. Do this even if image quality looks fine.
- Tracking-recovered event: notify that camera with exactly "Camera N, tracking recovered, carry on."
- Always finish with one log_decision call that states the decision and the deciding values.
Be brief. Do not invent values or thresholds; only cite numbers returned by tools or given in
the event. The only overlap thresholds are min_inliers and min_spread (see confidence_rule)."""

TOOLS = [
    ("get_quality", "Latest OpenCV 5 quality summary for one camera (rolling window medians).",
     {"camera": {"type": "integer"}}, ["camera"]),
    ("get_overlaps", "Recent overlaps between cameras with their verification state.", {}, []),
    ("verify_overlap", "Re-verify an overlap with OpenCV 5 (ORB + MAGSAC fundamental matrix) on the two "
     "keyframe images. Returns inliers, spread (image share the inliers cover) and confidence high/low.",
     {"overlap_id": {"type": "integer"}}, ["overlap_id"]),
    ("mark_covered", "Mark the overlap's place as covered. Refused for low-confidence overlaps "
     "until a human approves.", {"overlap_id": {"type": "integer"}}, ["overlap_id"]),
    ("request_approval", "Ask a human to approve a low-confidence overlap before coverage changes.",
     {"overlap_id": {"type": "integer"}, "reason": {"type": "string"}}, ["overlap_id", "reason"]),
    ("notify", "Send a message to one camera's walker.",
     {"camera": {"type": "integer"}, "message": {"type": "string"}}, ["camera", "message"]),
    ("log_decision", "Record the decision and the values it was based on.",
     {"camera": {"type": "integer"}, "decision": {"type": "string"}, "reason": {"type": "string"}},
     ["camera", "decision", "reason"]),
]
TOOL_CONFIG = {"tools": [{"toolSpec": {"name": n, "description": d, "inputSchema": {"json": {
    "type": "object", "properties": p, "required": req}}}} for n, d, p, req in TOOLS]}


class Agent:
    def __init__(self, tools, model_id="us.amazon.nova-2-lite-v1:0", region="us-east-1", max_turns=6):
        self.tools = tools  # name -> callable(**input) -> dict
        # Explicit timeouts: a dropped connection once hung a replay for 17 min in an SSL read.
        # A failed call raises, and the coordinator falls back to the rules for that event.
        self.client = boto3.client("bedrock-runtime", region_name=region, config=Config(
            connect_timeout=10, read_timeout=30, retries={"max_attempts": 3, "mode": "standard"}))
        self.model_id, self.max_turns = model_id, max_turns
        self.usage = {"inputTokens": 0, "outputTokens": 0, "calls": 0}

    def handle(self, event_text):
        """Run one event through the model until it stops calling tools. Returns the tool trace."""
        messages = [{"role": "user", "content": [{"text": event_text}]}]
        trace = []
        for _ in range(self.max_turns):
            r = self.client.converse(modelId=self.model_id, system=[{"text": SYSTEM}], messages=messages,
                                     toolConfig=TOOL_CONFIG, inferenceConfig={"maxTokens": 400, "temperature": 0})
            self.usage["calls"] += 1
            for k in ("inputTokens", "outputTokens"):
                self.usage[k] += r["usage"][k]
            msg = r["output"]["message"]
            messages.append(msg)
            uses = [b["toolUse"] for b in msg["content"] if "toolUse" in b]
            if not uses:
                break
            results = []
            for u in uses:
                fn = self.tools.get(u["name"])
                try:
                    out = fn(**u["input"]) if fn else {"error": f"unknown tool {u['name']}"}
                except Exception as e:  # a bad argument from the model must not kill the coordinator
                    out = {"error": str(e)}
                trace.append({"tool": u["name"], "input": u["input"], "output": out})
                results.append({"toolResult": {"toolUseId": u["toolUseId"], "content": [{"json": out}]}})
            messages.append({"role": "user", "content": results})
        return trace
