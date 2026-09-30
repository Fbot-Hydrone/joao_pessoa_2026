// -----------------------------------------------------------------------------
// maze_viewer — top-down debug view of the confined-space navigation stack
// -----------------------------------------------------------------------------
// Listens for JSON datagrams on UDP (default 47800) from anything that speaks
// hydrone_confined/core/viz_udp.py: the kinematic sim, the ROS viz bridge, a
// bag replay. Draws the known map, the true and estimated pose, the beams,
// the guard sectors, the particles, the path and the QR codes.
//
//   ./build/maze_viewer [port]
//
// Mouse wheel: zoom. Right/middle drag: pan. F: follow the drone. Home: fit.
// 1 beams, 2 sectors, 3 particles, 4 path, 5 trails, 6 rooms. C clears trails.
// F12 saves screenshot.png. MV_SHOT=<png> [MV_SHOT_AFTER=s] saves one and exits.
// State shows up in the window title (phase, pose source, QR count, error).
#define GLM_ENABLE_EXPERIMENTAL

#include <glad/glad.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>

#include "core/Window.h"
#include "render/camera.h"
#include "render/shaderClass.h"
#include "util/DebugHooks.h"

#include "json.h"

namespace
{

struct Col { float r, g, b, a; };

const Col BG{ 0.11f, 0.12f, 0.14f, 1.0f };
const Col WALL{ 0.85f, 0.85f, 0.82f, 1.0f };
const Col WALL_OUT{ 0.95f, 0.95f, 0.92f, 1.0f };
const Col ROOM{ 0.35f, 0.45f, 0.65f, 0.6f };
const Col STRUCT{ 0.25f, 0.27f, 0.30f, 1.0f };
const Col ARENA{ 0.30f, 0.30f, 0.30f, 1.0f };
const Col PAD{ 0.20f, 0.35f, 0.75f, 0.8f };
const Col HOLE{ 1.0f, 0.60f, 0.15f, 1.0f };
const Col WINDOW{ 0.30f, 0.85f, 0.95f, 1.0f };
const Col DOOR{ 0.40f, 0.55f, 1.0f, 1.0f };
const Col FORBID{ 0.95f, 0.25f, 0.25f, 1.0f };
const Col ENTRY{ 0.30f, 0.95f, 0.40f, 1.0f };
const Col EXIT{ 0.95f, 0.35f, 0.95f, 1.0f };
const Col TRUTH{ 0.30f, 0.95f, 0.40f, 1.0f };
const Col EST{ 0.35f, 0.65f, 1.0f, 1.0f };
const Col BEAM{ 0.95f, 0.85f, 0.30f, 0.55f };
const Col SECTOR{ 0.95f, 0.35f, 0.30f, 0.8f };
const Col PARTICLE{ 0.75f, 0.55f, 1.0f, 0.7f };
const Col PATH{ 1.0f, 0.90f, 0.20f, 1.0f };
const Col QR_OFF{ 0.60f, 0.45f, 0.20f, 1.0f };
const Col QR_ON{ 1.0f, 0.65f, 0.0f, 1.0f };

// Vertices (x, y, z, r, g, b, a) rebuilt every frame and drawn with one call per mode.
class Batch
{
public:
	std::vector<float> v;
	GLuint vao = 0, vbo = 0;

	void init()
	{
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);
		glBindVertexArray(vao);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
		glBindVertexArray(0);
	}
	void add(float x, float y, float z, Col c)
	{
		v.insert(v.end(), { x, y, z, c.r, c.g, c.b, c.a });
	}
	void draw(GLenum mode)
	{
		if (v.empty()) return;
		glBindVertexArray(vao);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STREAM_DRAW);
		glDrawArrays(mode, 0, (GLsizei)(v.size() / 7));
		glBindVertexArray(0);
		v.clear();
	}
	void del()
	{
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
	}
};

Batch tris, lines, points;

void line(float x0, float y0, float x1, float y1, Col c, float z = 0.0f)
{
	lines.add(x0, y0, z, c);
	lines.add(x1, y1, z, c);
}

void quad(float ax, float ay, float bx, float by, float half, Col c, float z = 0.0f)
{
	// a thick segment a-b, `half` either side
	float dx = bx - ax, dy = by - ay;
	float L = std::sqrt(dx * dx + dy * dy);
	if (L < 1e-6f) return;
	float nx = -dy / L * half, ny = dx / L * half;
	tris.add(ax + nx, ay + ny, z, c); tris.add(bx + nx, by + ny, z, c); tris.add(bx - nx, by - ny, z, c);
	tris.add(ax + nx, ay + ny, z, c); tris.add(bx - nx, by - ny, z, c); tris.add(ax - nx, ay - ny, z, c);
}

void rectOutline(float x0, float x1, float y0, float y1, Col c, float z = 0.0f)
{
	line(x0, y0, x1, y0, c, z); line(x1, y0, x1, y1, c, z);
	line(x1, y1, x0, y1, c, z); line(x0, y1, x0, y0, c, z);
}

void rectFill(float x0, float x1, float y0, float y1, Col c, float z = 0.0f)
{
	tris.add(x0, y0, z, c); tris.add(x1, y0, z, c); tris.add(x1, y1, z, c);
	tris.add(x0, y0, z, c); tris.add(x1, y1, z, c); tris.add(x0, y1, z, c);
}

void circle(float cx, float cy, float r, Col c, float z = 0.0f, int n = 32)
{
	for (int k = 0; k < n; k++)
	{
		float a0 = 2.0f * (float)M_PI * k / n, a1 = 2.0f * (float)M_PI * (k + 1) / n;
		line(cx + r * std::cos(a0), cy + r * std::sin(a0), cx + r * std::cos(a1), cy + r * std::sin(a1), c, z);
	}
}

void poseMarker(double x, double y, double yaw, Col c, float z)
{
	// drone disc (0.165 m, the Kopis with guards) + a nose
	circle((float)x, (float)y, 0.165f, c, z);
	float cx = (float)x, cy = (float)y, cs = (float)std::cos(yaw), sn = (float)std::sin(yaw);
	tris.add(cx + 0.22f * cs, cy + 0.22f * sn, z, c);
	tris.add(cx - 0.08f * cs - 0.09f * sn, cy - 0.08f * sn + 0.09f * cs, z, c);
	tris.add(cx - 0.08f * cs + 0.09f * sn, cy - 0.08f * sn - 0.09f * cs, z, c);
}

Col openingColor(const Json& o)
{
	std::string role = o["role"].s(), kind = o["kind"].s();
	if (role == "forbidden") return FORBID;
	if (role == "entry") return ENTRY;
	if (role == "exit") return EXIT;
	if (kind == "hole") return HOLE;
	if (kind == "door") return DOOR;
	return WINDOW;
}

struct View
{
	float cx = 5.0f, cy = 2.0f, half = 3.0f;
	bool follow = false;
	bool beams = true, sectors = true, particles = true, path = true, trails = true, rooms = true;
};

void drawMap(const Json& m, const View& view)
{
	const Json& a = m["arena"];
	if (a.size() == 4) rectOutline((float)a[0].d(), (float)a[1].d(), (float)a[2].d(), (float)a[3].d(), ARENA, -0.9f);
	const Json& s = m["structure"];
	if (s.size() == 4) rectFill((float)s[0].d(), (float)s[1].d(), (float)s[2].d(), (float)s[3].d(), STRUCT, -0.8f);
	const Json& to = m["takeoff"];
	if (to.has("center"))
	{
		float x = (float)to["center"][0].d(), y = (float)to["center"][1].d();
		float w = (float)to["size"][0].d(1.0) / 2, h = (float)to["size"][1].d(1.0) / 2;
		rectFill(x - w, x + w, y - h, y + h, PAD, -0.7f);
	}
	const Json& lp = m["landing"];
	if (lp.size() == 2)
	{
		float x = (float)lp[0].d(), y = (float)lp[1].d();
		rectFill(x - 0.5f, x + 0.5f, y - 0.5f, y + 0.5f, PAD, -0.7f);
		circle(x, y, 0.25f, WALL, -0.6f);
	}
	if (view.rooms)
		for (const Json& r : m["rooms"].arr)
			rectOutline((float)r["x"][0].d(), (float)r["x"][1].d(), (float)r["y"][0].d(), (float)r["y"][1].d(), ROOM, -0.5f);
	for (const Json& w : m["walls"].arr)
	{
		double ax = w["a"][0].d(), ay = w["a"][1].d(), bx = w["b"][0].d(), by = w["b"][1].d();
		double L = std::hypot(bx - ax, by - ay);
		if (L < 1e-6) continue;
		double tx = (bx - ax) / L, ty = (by - ay) / L;
		float half = (float)std::max(w["t"].d(0.06), 0.03) / 2;
		Col wc = w["kind"].s() == "outer" ? WALL_OUT : WALL;
		// solid where no opening reaches the floor-to-roof... draw all openings on top instead
		quad((float)ax, (float)ay, (float)bx, (float)by, half, wc, -0.3f);
		for (const Json& o : w["openings"].arr)
		{
			double u0 = o["u"][0].d(), u1 = o["u"][1].d();
			Col oc = openingColor(o);
			quad((float)(ax + tx * u0), (float)(ay + ty * u0), (float)(ax + tx * u1), (float)(ay + ty * u1),
			     half + 0.015f, oc, -0.2f);
			if (o["soft"].kind == Json::Bool && o["soft"].b)
				quad((float)(ax + tx * u0), (float)(ay + ty * u0), (float)(ax + tx * u1), (float)(ay + ty * u1),
				     half * 0.35f, BG, -0.15f);
		}
	}
}

void drawFrame(const Json& f, const View& view, const std::deque<std::pair<float, float>>& trueTrail,
               const std::deque<std::pair<float, float>>& estTrail)
{
	if (view.trails)
	{
		for (size_t k = 1; k < trueTrail.size(); k++)
			line(trueTrail[k - 1].first, trueTrail[k - 1].second, trueTrail[k].first, trueTrail[k].second,
			     { TRUTH.r, TRUTH.g, TRUTH.b, 0.45f }, 0.05f);
		for (size_t k = 1; k < estTrail.size(); k++)
			line(estTrail[k - 1].first, estTrail[k - 1].second, estTrail[k].first, estTrail[k].second,
			     { EST.r, EST.g, EST.b, 0.45f }, 0.06f);
	}
	if (view.particles)
		for (const Json& p : f["particles"].arr)
			points.add((float)p[0].d(), (float)p[1].d(), 0.1f, PARTICLE);
	const Json& sensor = f["sensor"];
	float sx = (float)sensor[0].d(), sy = (float)sensor[1].d();
	if (view.beams && sensor.size() == 2)
		for (const Json& b : f["beams"].arr)
		{
			double a = b[0].d(), r = b[1].d();
			if (r > 50.0) continue;
			line(sx, sy, sx + (float)(r * std::cos(a)), sy + (float)(r * std::sin(a)), BEAM, 0.2f);
			points.add(sx + (float)(r * std::cos(a)), sy + (float)(r * std::sin(a)), 0.25f, BEAM);
		}
	if (view.sectors && sensor.size() == 2)
		for (const Json& b : f["sectors"].arr)
		{
			double a = b[0].d(), r = b[1].d();
			if (r > 0.7) continue;          // only the ones the guard cares about
			double w = M_PI / std::max<size_t>(f["sectors"].size(), 1);
			line(sx + (float)(r * std::cos(a - w)), sy + (float)(r * std::sin(a - w)),
			     sx + (float)(r * std::cos(a + w)), sy + (float)(r * std::sin(a + w)), SECTOR, 0.3f);
		}
	if (view.path)
	{
		const Json& p = f["path"];
		for (size_t k = 1; k < p.size(); k++)
			line((float)p[k - 1][0].d(), (float)p[k - 1][1].d(), (float)p[k][0].d(), (float)p[k][1].d(), PATH, 0.35f);
		const Json& g = f["goal"];
		if (g.size() == 2)
		{
			float gx = (float)g[0].d(), gy = (float)g[1].d();
			line(gx - 0.08f, gy - 0.08f, gx + 0.08f, gy + 0.08f, PATH, 0.35f);
			line(gx - 0.08f, gy + 0.08f, gx + 0.08f, gy - 0.08f, PATH, 0.35f);
		}
	}
	const Json& qr = f["qr_world"];
	std::vector<std::string> seen;
	for (const Json& q : f["qr_seen"].arr) seen.push_back(q.s());
	for (const auto& kv : qr.obj)
	{
		bool on = std::find(seen.begin(), seen.end(), kv.first) != seen.end();
		float x = (float)kv.second[0].d(), y = (float)kv.second[1].d();
		rectFill(x - 0.06f, x + 0.06f, y - 0.06f, y + 0.06f, on ? QR_ON : QR_OFF, 0.4f);
	}
	const Json& tr = f["truth"];
	if (tr.size() >= 4) poseMarker(tr[0].d(), tr[1].d(), tr[3].d(), TRUTH, 0.5f);
	const Json& es = f["est"];
	if (es.size() >= 4) poseMarker(es[0].d(), es[1].d(), es[3].d(), EST, 0.6f);
}

void fit(View& view, const Json& map, int w, int h)
{
	const Json& s = map["structure"].size() == 4 ? map["structure"] : map["arena"];
	if (s.size() != 4) return;
	view.cx = (float)(s[0].d() + s[1].d()) / 2;
	view.cy = (float)(s[2].d() + s[3].d()) / 2;
	// half is vertical: the width has to fit through the window's aspect too
	double aspect = (double)std::max(h, 10) / (double)std::max(w, 10);
	view.half = (float)(std::max((s[3].d() - s[2].d()) / 2, (s[1].d() - s[0].d()) / 2 * aspect) * 1.15 + 0.3);
}

int openSocket(int port)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) return -1;
	int yes = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
	int big = 1 << 22;
	setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &big, sizeof(big));
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0)
	{
		close(fd);
		return -1;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	return fd;
}

} // namespace

int main(int argc, char* argv[])
{
	int port = argc > 1 ? std::atoi(argv[1]) : 47800;
	int fd = openSocket(port);
	if (fd < 0)
	{
		std::fprintf(stderr, "maze_viewer: can't bind UDP port %d\n", port);
		return 1;
	}
	SDLState state = initialize(1500, 800, { .title = "maze viewer", .resizable = true, .vsync = true });
	if (!state.successfullyInitialized) return 1;
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_PROGRAM_POINT_SIZE);

	Shader shader(MV_SHADER_DIR "flat.vert", MV_SHADER_DIR "flat.frag");
	Camera camera(glm::vec3(5.0f, 2.0f, 10.0f));
	camera.orientation = glm::vec3(0.0f, 0.0f, -1.0f);
	camera.up = glm::vec3(0.0f, 1.0f, 0.0f);
	tris.init();
	lines.init();
	points.init();

	View view;
	Json map, frame;
	std::deque<std::pair<float, float>> trueTrail, estTrail;
	std::vector<char> buf(1 << 16);
	bool dragging = false;
	const char* shot = std::getenv("MV_SHOT");
	float shotAfter = std::getenv("MV_SHOT_AFTER") ? (float)std::atof(std::getenv("MV_SHOT_AFTER")) : 3.0f;
	uint64_t t0 = SDL_GetTicks();
	uint64_t lastMsg = 0;
	bool running = true;
	bool wantShot = false;

	while (running)
	{
		// everything that arrived since the last frame; only the newest frame matters
		for (;;)
		{
			ssize_t n = recv(fd, buf.data(), buf.size() - 1, 0);
			if (n <= 0) break;
			Json msg = Json::parse(std::string(buf.data(), (size_t)n));
			std::string type = msg["type"].s();
			lastMsg = SDL_GetTicks();
			if (type == "map")
			{
				bool first = map.isNull();
				map = msg;
				if (first) fit(view, map, state.width, state.height);
			}
			else if (type == "frame")
			{
				frame = msg;
				const Json& tr = msg["truth"];
				if (tr.size() >= 2)
				{
					trueTrail.emplace_back((float)tr[0].d(), (float)tr[1].d());
					if (trueTrail.size() > 4000) trueTrail.pop_front();
				}
				const Json& es = msg["est"];
				if (es.size() >= 2)
				{
					estTrail.emplace_back((float)es[0].d(), (float)es[1].d());
					if (estTrail.size() > 4000) estTrail.pop_front();
				}
			}
			else if (type == "reset")
			{
				trueTrail.clear();
				estTrail.clear();
				frame = Json();
			}
		}

		SDL_Event e;
		while (SDL_PollEvent(&e))
		{
			switch (e.type)
			{
			case SDL_EVENT_QUIT:
				running = false;
				break;
			case SDL_EVENT_WINDOW_RESIZED:
			{
				int w = 0, h = 0;
				SDL_GetWindowSizeInPixels(state.window, &w, &h);
				state.width = std::max(w, 10);
				state.height = std::max(h, 10);
				glViewport(0, 0, state.width, state.height);
				break;
			}
			case SDL_EVENT_MOUSE_WHEEL:
				view.half *= e.wheel.y > 0 ? 0.88f : 1.0f / 0.88f;
				view.half = std::clamp(view.half, 0.3f, 60.0f);
				break;
			case SDL_EVENT_MOUSE_BUTTON_DOWN:
				if (e.button.button == SDL_BUTTON_RIGHT || e.button.button == SDL_BUTTON_MIDDLE) dragging = true;
				break;
			case SDL_EVENT_MOUSE_BUTTON_UP:
				dragging = false;
				break;
			case SDL_EVENT_MOUSE_MOTION:
				if (dragging)
				{
					float perPx = 2.0f * view.half / (float)state.height;
					view.cx -= e.motion.xrel * perPx;
					view.cy += e.motion.yrel * perPx;
					view.follow = false;
				}
				break;
			case SDL_EVENT_KEY_DOWN:
				switch (e.key.scancode)
				{
				case SDL_SCANCODE_F: view.follow = !view.follow; break;
				case SDL_SCANCODE_HOME: fit(view, map, state.width, state.height); view.follow = false; break;
				case SDL_SCANCODE_1: view.beams = !view.beams; break;
				case SDL_SCANCODE_2: view.sectors = !view.sectors; break;
				case SDL_SCANCODE_3: view.particles = !view.particles; break;
				case SDL_SCANCODE_4: view.path = !view.path; break;
				case SDL_SCANCODE_5: view.trails = !view.trails; break;
				case SDL_SCANCODE_6: view.rooms = !view.rooms; break;
				case SDL_SCANCODE_C: trueTrail.clear(); estTrail.clear(); break;
				case SDL_SCANCODE_F12: wantShot = true; break;
				case SDL_SCANCODE_ESCAPE: running = false; break;
				default: break;
				}
				break;
			default:
				break;
			}
		}

		if (view.follow && frame["est"].size() >= 2)
		{
			view.cx = (float)frame["est"][0].d();
			view.cy = (float)frame["est"][1].d();
		}
		camera.position = glm::vec3(view.cx, view.cy, 10.0f);
		camera.updateMatrixOrtho(view.half, 0.1f, 50.0f, state.width, state.height);

		glClearColor(BG.r, BG.g, BG.b, BG.a);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		shader.Activate();
		camera.Matrix(shader, "camMatrix");
		glUniform1f(glGetUniformLocation(shader.ID, "pointSize"), 4.0f);

		if (!map.isNull()) drawMap(map, view);
		if (!frame.isNull()) drawFrame(frame, view, trueTrail, estTrail);
		tris.draw(GL_TRIANGLES);
		lines.draw(GL_LINES);
		points.draw(GL_POINTS);

		// the title is the status line
		char title[256];
		if (frame.isNull())
			std::snprintf(title, sizeof(title), "maze viewer | waiting on UDP %d%s", port,
			              map.isNull() ? "" : " (map received)");
		else
		{
			double err = -1.0;
			const Json &tr = frame["truth"], &es = frame["est"];
			if (tr.size() >= 2 && es.size() >= 2) err = std::hypot(tr[0].d() - es[0].d(), tr[1].d() - es[1].d());
			std::snprintf(title, sizeof(title), "maze viewer | t %.1f  %s | source %s | QR %zu | err %.2f m | %s%s",
			              frame["t"].d(), frame["phase"].s().c_str(), frame["mode"].s("?").c_str(),
			              frame["qr_seen"].size(), err, frame["note"].s().c_str(),
			              SDL_GetTicks() - lastMsg > 2000 ? " | (stale)" : "");
		}
		SDL_SetWindowTitle(state.window, title);

		// read the back buffer before the swap leaves it undefined
		if (wantShot)
		{
			saveScreenshot(state, "screenshot.png");
			wantShot = false;
		}
		if (shot && (SDL_GetTicks() - t0) / 1000.0f > shotAfter)
		{
			saveScreenshot(state, shot);
			running = false;
		}
		SDL_GL_SwapWindow(state.window);
	}
	tris.del();
	lines.del();
	points.del();
	shader.Delete();
	close(fd);
	cleanup(state);
	return 0;
}
