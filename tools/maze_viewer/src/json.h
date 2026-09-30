#pragma once
// -----------------------------------------------------------------------------
// json.h — just enough JSON to read the viewer's UDP messages
// -----------------------------------------------------------------------------
// Objects, arrays, numbers, strings (no \u escapes beyond ASCII), true/false/null.
// Parse failures give a Null value instead of throwing.

#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

struct Json
{
	enum Kind { Null, Bool, Number, String, Array, Object } kind = Null;
	double num = 0.0;
	bool b = false;
	std::string str;
	std::vector<Json> arr;
	std::map<std::string, Json> obj;

	bool isNull() const { return kind == Null; }
	const Json& operator[](const std::string& key) const
	{
		static const Json none;
		if (kind != Object) return none;
		auto it = obj.find(key);
		return it == obj.end() ? none : it->second;
	}
	const Json& operator[](size_t i) const
	{
		static const Json none;
		return (kind == Array && i < arr.size()) ? arr[i] : none;
	}
	size_t size() const { return kind == Array ? arr.size() : (kind == Object ? obj.size() : 0); }
	double d(double fallback = 0.0) const { return kind == Number ? num : fallback; }
	std::string s(const std::string& fallback = "") const { return kind == String ? str : fallback; }
	bool has(const std::string& key) const { return kind == Object && obj.count(key); }

	static Json parse(const std::string& text)
	{
		size_t i = 0;
		Json out;
		if (!value(text, i, out)) return Json();
		return out;
	}

private:
	static void ws(const std::string& t, size_t& i)
	{
		while (i < t.size() && (t[i] == ' ' || t[i] == '\n' || t[i] == '\r' || t[i] == '\t')) i++;
	}

	static bool value(const std::string& t, size_t& i, Json& out)
	{
		ws(t, i);
		if (i >= t.size()) return false;
		char c = t[i];
		if (c == '{')
		{
			out.kind = Object;
			i++;
			ws(t, i);
			if (i < t.size() && t[i] == '}') { i++; return true; }
			while (i < t.size())
			{
				Json key;
				if (!string(t, i, key)) return false;
				ws(t, i);
				if (i >= t.size() || t[i] != ':') return false;
				i++;
				Json v;
				if (!value(t, i, v)) return false;
				out.obj[key.str] = std::move(v);
				ws(t, i);
				if (i < t.size() && t[i] == ',') { i++; continue; }
				if (i < t.size() && t[i] == '}') { i++; return true; }
				return false;
			}
			return false;
		}
		if (c == '[')
		{
			out.kind = Array;
			i++;
			ws(t, i);
			if (i < t.size() && t[i] == ']') { i++; return true; }
			while (i < t.size())
			{
				Json v;
				if (!value(t, i, v)) return false;
				out.arr.push_back(std::move(v));
				ws(t, i);
				if (i < t.size() && t[i] == ',') { i++; continue; }
				if (i < t.size() && t[i] == ']') { i++; return true; }
				return false;
			}
			return false;
		}
		if (c == '"') return string(t, i, out);
		if (t.compare(i, 4, "true") == 0) { out.kind = Bool; out.b = true; i += 4; return true; }
		if (t.compare(i, 5, "false") == 0) { out.kind = Bool; out.b = false; i += 5; return true; }
		if (t.compare(i, 4, "null") == 0) { out.kind = Null; i += 4; return true; }
		// NaN / Infinity as Python's json writes them: read as a huge number
		if (t.compare(i, 8, "Infinity") == 0) { out.kind = Number; out.num = 1e30; i += 8; return true; }
		if (t.compare(i, 9, "-Infinity") == 0) { out.kind = Number; out.num = -1e30; i += 9; return true; }
		if (t.compare(i, 3, "NaN") == 0) { out.kind = Null; i += 3; return true; }
		char* end = nullptr;
		double v = std::strtod(t.c_str() + i, &end);
		if (end == t.c_str() + i) return false;
		out.kind = Number;
		out.num = v;
		i = end - t.c_str();
		return true;
	}

	static bool string(const std::string& t, size_t& i, Json& out)
	{
		ws(t, i);
		if (i >= t.size() || t[i] != '"') return false;
		i++;
		out.kind = String;
		while (i < t.size() && t[i] != '"')
		{
			if (t[i] == '\\' && i + 1 < t.size())
			{
				char e = t[i + 1];
				out.str += (e == 'n') ? '\n' : (e == 't') ? '\t' : e;
				i += 2;
				continue;
			}
			out.str += t[i++];
		}
		if (i >= t.size()) return false;
		i++;
		return true;
	}
};
