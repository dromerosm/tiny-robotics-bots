// GET /live (a WebSocket, with the header X-Robot-Token): the robot's talk
// with GPT-Live, through this Worker.
//
// GPT-Live has no short-lived keys for devices, so the Worker opens the
// session to OpenAI itself, with our API key, and passes the messages both
// ways. The robot never sees the key. The Worker also starts the session
// with our settings, so the robot can't change the model, the voice or the
// delegation:
// - gpt-live-1 talks and listens, full duplex;
// - when it needs to look something up or think, it hands the task to
//   gpt-6-luna (Responses delegation), with web search. Add its tools in
//   SESSION.delegation.responses.tools.
//
// From the robot it passes only audio (session.input_audio.append) and
// session.close. To the robot it passes everything except the delegated
// model's own events (response.event): they are big, and the robot does not
// need them.
const SESSION = {
  model: "gpt-live-1",
  // In Spanish, so the model stays in Spanish.
  instructions:
    "Eres StackChan, un robot pequeño que vive en un escritorio. " +
    "Habla siempre en español de España, aunque te hablen en otro idioma o no entiendas lo que oyes. " +
    "Nunca cambies de idioma. Responde corto: una o dos frases. " +
    "Si te preguntan algo de hoy (tiempo, noticias, horarios, precios), algo que hay que buscar o calcular, " +
    "o algo que no sabes seguro, delega la tarea.",
  // 16 kHz, the robot's own rate: no conversion on the robot.
  audio: { format: { type: "audio/pcm", rate: 16000 }, output: { voice: "vesper" } },
  delegation: {
    type: "responses",
    responses: {
      model: "gpt-6-luna",
      instructions:
        "Tu respuesta la dirá en voz alta un robot. En español de España, en dos frases como mucho, " +
        "sin enlaces, sin listas y sin markdown.",
      tools: [{ type: "web_search" }],
      tool_choice: "auto",
    },
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
    if (url.pathname !== "/live" || request.headers.get("Upgrade") !== "websocket") {
      return new Response("not found\n", { status: 404 });
    }
    const token = request.headers.get("X-Robot-Token") || "";
    if (!env.ROBOT_TOKEN || !same(token, env.ROBOT_TOKEN)) {
      return new Response("forbidden\n", { status: 403 });
    }

    const r = await fetch("https://api.openai.com/v1/live/sessions", {
      headers: { Upgrade: "websocket", Authorization: `Bearer ${env.OPENAI_API_KEY}` },
    });
    const openai = r.webSocket;
    if (!openai) {
      console.log("OpenAI said", r.status, await r.text());
      return new Response(`OpenAI said ${r.status}\n`, { status: 502 });
    }
    openai.accept();
    openai.send(JSON.stringify({ type: "session.start", session: SESSION }));

    const [client, robot] = Object.values(new WebSocketPair());
    robot.accept();

    robot.addEventListener("message", (e) => {
      if (typeof e.data !== "string") {
        return;
      }
      let type;
      try {
        type = JSON.parse(e.data).type;
      } catch {
        return;
      }
      if (type === "session.input_audio.append" || type === "session.close") {
        openai.send(e.data);
      }
    });
    openai.addEventListener("message", (e) => {
      if (typeof e.data !== "string" || e.data.includes('"type": "response.event"') ||
          e.data.includes('"type":"response.event"')) {
        return;
      }
      try {
        robot.send(e.data);
      } catch {
        // the robot is gone
      }
    });
    const close = (ws, code, reason) => {
      try {
        ws.close(code === 1005 || code === 1006 ? 1000 : code, reason);
      } catch {
        // already closed
      }
    };
    robot.addEventListener("close", (e) => close(openai, e.code, e.reason));
    openai.addEventListener("close", (e) => close(robot, e.code, e.reason));
    robot.addEventListener("error", () => close(openai, 1011, "robot error"));
    openai.addEventListener("error", () => close(robot, 1011, "OpenAI error"));

    return new Response(null, { status: 101, webSocket: client });
  },
};
