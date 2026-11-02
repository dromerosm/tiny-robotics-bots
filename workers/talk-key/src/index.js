// POST /key with the header X-Robot-Token: a Realtime session key that
// works for 60 s, for one session with our settings. The robot never sees
// the OpenAI API key, and it can't change the model or the settings.
const MODEL = "gpt-realtime-2.1";

const SESSION = {
  type: "realtime",
  model: MODEL,
  instructions:
    "You are StackChan, a small robot on a desk. You see the person through a camera " +
    "and turn your head to follow their face. Speak Spanish unless the person speaks " +
    "another language. Keep answers short: one or two sentences.",
  audio: {
    input: {
      format: { type: "audio/pcm", rate: 24000 },
      // The server hears when the person starts and stops talking. When the
      // person talks over the robot, the answer stops (full duplex).
      turn_detection: { type: "server_vad", create_response: true, interrupt_response: true },
    },
    output: { format: { type: "audio/pcm", rate: 24000 }, voice: "marin" },
  },
};

function same(a, b) {
  const x = new TextEncoder().encode(a);
  const y = new TextEncoder().encode(b);
  return x.length === y.length && crypto.subtle.timingSafeEqual(x, y);
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    if (request.method !== "POST" || url.pathname !== "/key") {
      return new Response("not found\n", { status: 404 });
    }
    const token = request.headers.get("X-Robot-Token") || "";
    if (!env.ROBOT_TOKEN || !same(token, env.ROBOT_TOKEN)) {
      return new Response("forbidden\n", { status: 403 });
    }
    const r = await fetch("https://api.openai.com/v1/realtime/client_secrets", {
      method: "POST",
      headers: {
        Authorization: `Bearer ${env.OPENAI_API_KEY}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ expires_after: { anchor: "created_at", seconds: 60 }, session: SESSION }),
    });
    if (!r.ok) {
      console.log("OpenAI said", r.status, await r.text());
      return new Response(`OpenAI said ${r.status}\n`, { status: 502 });
    }
    const d = await r.json();
    // Plain text, two lines, easy to read on the robot: the model, the key.
    return new Response(`${MODEL}\n${d.value}\n`, {
      headers: { "Content-Type": "text/plain", "Cache-Control": "no-store" },
    });
  },
};
