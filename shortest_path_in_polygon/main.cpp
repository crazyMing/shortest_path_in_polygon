#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <stack>
#include "interface.h"

typedef struct _point
{
	double x;
	double y;
	_point() : x(0.0), y(0.0) {}
	_point(double xp, double yp) : x(xp), y(yp){}
} point, vect;

typedef std::vector<point> points;
typedef std::vector<int> path;
typedef std::pair<int, int> diagnal;

points polygon_points;
const char *usage = "Usage: shortest_path_in_polygon.exe [room_map_file_path] [path_file_path] [output_format]";

inline double dist(const point &p, const point &q);
path simple_stupid_funnel(const std::vector<diagnal> &d_list, const point &start, const point &end);
inline int sign(double v);
inline double cross_product(const vect &v1, const vect &v2);
std::vector<diagnal> diagnals_list(const int triangles[][3], const std::vector<int> &triangles_list);
std::vector<int> dfs(const int triangles[][3], int triangles_count, int start_tri, int end_tri);
bool neighbor_triangles(const int triangles[][3], int triangles_count,
	int tri1, int tri2);
int locate_point(const int triangles[][3], int triangles_count, const point &k);
bool in_triangle(const point &p, const point &q, const point &r, const point &k);
bool to_left(const point &p, const point &q, const point &k);

int main(int argc, char *argv[])
{
	if (argc != 4)
	{
		std::cout << usage << std::endl;
		return -1;
	}

	std::string line;
	std::ifstream input(argv[1], std::ifstream::in);

	std::getline(input, line);
	std::istringstream seiss(line);
	double sx, sy, ex, ey;
	seiss >> sx >> sy >> ex >> ey;
	point start(sx, sy), end(ex, ey);

	std::getline(input, line);
	std::istringstream niss(line);
	int count = 0;
	niss >> count;
	for (int i = 0; i < count; i++)
	{
		std::getline(input, line);
		std::istringstream iss(line);
		double x, y;
		iss >> x >> y;
		polygon_points.push_back(point(x, y));
	}

	int vertices_count = polygon_points.size();
	int triangles_count = polygon_points.size() - 2;

	double (*vertices)[2] = new double[vertices_count + 1][2];
	int (*triangles)[3] = new int[triangles_count][3];

	for (int i = 1; i <= polygon_points.size(); i++)
	{
		vertices[i][0] = polygon_points[i - 1].x;
		vertices[i][1] = polygon_points[i - 1].y;
	}
	int ncontours = 1;
	int cntr[1] = {polygon_points.size()};
	triangulate_polygon(ncontours, cntr, vertices, triangles);

	for (int i = 0; i < triangles_count; i++)
	{
		triangles[i][0]--;
		triangles[i][1]--;
		triangles[i][2]--;
	}
	std::ofstream output(argv[2], std::ofstream::out);
	int start_tri = locate_point(triangles, triangles_count, start);
	int end_tri = locate_point(triangles, triangles_count, end);
	bool distance = false;
	if (strcmp(argv[3], "path") == 0)
		distance = false;
	else if (strcmp(argv[3], "distance") == 0)
		distance = true;
	if (start_tri == end_tri)
	{
		if (distance)
			output << (int)ceil(dist(start, end)) << std::endl;
		else
		{
			output << "s" << std::endl;
			output << "e" << std::endl;
		}
	}
	else
	{
		std::vector<int> triangles_list = dfs(triangles, triangles_count, start_tri, end_tri);
		std::vector<diagnal> d_list = diagnals_list(triangles, triangles_list);
		path shortest_path = simple_stupid_funnel(d_list, start, end);

		if (distance)
		{
			double total_dist = 0;
			for (int i = 0; i < shortest_path.size(); i++)
			{
				point last;
				point current = polygon_points[shortest_path[i]];
				if (i == 0)
					last = start;
				else
					last = polygon_points[shortest_path[i - 1]];
				total_dist += dist(last, current);
			}
			if (shortest_path.size() > 0)
				total_dist += dist(polygon_points[shortest_path[shortest_path.size() - 1]], end);
			else
				total_dist += dist(start, end);
			output << (int)ceil(total_dist) << std::endl;
		}
		else
		{
			output << "s" << std::endl;
			for (int i = 0; i < shortest_path.size(); i++)
				output << shortest_path[i] << std::endl;
			output << "e" << std::endl;
		}
	}

	delete[] vertices;
	delete[] triangles;
	return 0;
}

inline double dist(const point &p, const point &q)
{
	return sqrt((p.x - q.x)*(p.x - q.x) + (p.y - q.y)*(p.y - q.y));
}

path simple_stupid_funnel(const std::vector<diagnal> &d_list, const point &start, const point &end)
{
	path shortest_path;
	point apex = start;
	int apex_id = -1;
	int left = d_list[0].first;
	int right = d_list[0].second;
	int lo = left;
	int ro = right;
	int left_id = 0;
	int right_id = 0;
	bool add_left = false;
	for (int i = 1; i <= d_list.size(); i++)
	{
		vect lv, rv, nv;
		lv = vect(polygon_points[left].x - apex.x, polygon_points[left].y - apex.y);
		rv = vect(polygon_points[right].x - apex.x, polygon_points[right].y - apex.y);
		int new_p = -1;
		bool last_end = i == d_list.size();
		if (last_end)
			nv = vect(end.x - apex.x, end.y - apex.y);
		else
		{
			if (d_list[i].first == left || d_list[i].first == lo)
			{
				new_p = d_list[i].second;
				add_left = false;
			}
			else if (d_list[i].second == left || d_list[i].second == lo)
			{
				new_p = d_list[i].first;
				add_left = false;
			}
			else if (d_list[i].first == right || d_list[i].first == ro)
			{
				new_p = d_list[i].second;
				add_left = true;
			}
			else if (d_list[i].second == right || d_list[i].second == ro)
			{
				new_p = d_list[i].first;
				add_left = true;
			}
			nv = vect(polygon_points[new_p].x - apex.x, polygon_points[new_p].y - apex.y);
		}
#ifdef _DEBUG
		if (apex_id == -1)
			std::cout << "Funel: [Apex: start, ";
		else
			std::cout << "Funel: [Apex:" << apex_id << ", ";
		std::cout << "Left:" << left << ", Right:" << right << "]" << std::endl;
#endif

		bool cond1 = sign(cross_product(lv, nv)) == sign(cross_product(lv, rv));
		bool cond2 = sign(cross_product(rv, nv)) == sign(cross_product(rv, lv));
		if (add_left || last_end)
		{	
			if (cond1)
			{
				if (cond2)
				{
#ifdef _DEBUG
					std::cout << "Left:" << left << " contracts to ";
					if (new_p == -1)
						std::cout << "end" << std::endl;
					else
						std::cout << new_p << std::endl;
#endif
					left = new_p;
					left_id = i;
				}
				else
				{
#ifdef _DEBUG
					std::cout << "Add " << right << " to shortes path (New apex)" << std::endl;
#endif
					shortest_path.push_back(right);
					apex = polygon_points[right];
					apex_id = right;
					for (i = right_id; i < d_list.size(); i++)
					{
						if (d_list[i].first != right && d_list[i].second != right)
						{
							left = d_list[i].first;
							right = d_list[i].second;
							left_id = right_id = i;
#ifdef _DEBUG
							std::cout << "Update Left:" << left << " Right:" << right << std::endl;
#endif
							break;
						}
					}
				}
			}
			else
				lo = new_p;
		}
		if (!add_left || last_end)
		{
			if (cond2)
			{
				if (cond1)
				{
#ifdef _DEBUG
					std::cout << "Right:" << right << " contracts to ";
					if (new_p == -1)
						std::cout << "end" << std::endl;
					else
						std::cout << new_p << std::endl;
#endif
					right = new_p;
					right_id = i;
				}
				else
				{
#ifdef _DEBUG
					std::cout << "Add " << right << " to shortes path (New apex)" << std::endl;
#endif
					shortest_path.push_back(left);
					apex = polygon_points[left];
					apex_id = left;
					for (i = left_id; i < d_list.size(); i++)
					{
						if (d_list[i].first != left && d_list[i].second != left)
						{
							left = d_list[i].first;
							right = d_list[i].second;
							left_id = right_id = i;
#ifdef _DEBUG
							std::cout << "Update Left:" << left << " Right:" << right << std::endl;
#endif
							break;
						}
					}
				}
			}
			else
				ro = new_p;
		}
	}
	return shortest_path;
}

inline int sign(double v)
{
	if (v > 0) return 1;
	else if (v < 0) return -1;
	else return 0;
}

inline double cross_product(const vect &v1, const vect &v2)
{
	return v1.x * v2.y - v2.x * v1.y;
}

std::vector<diagnal> diagnals_list(const int triangles[][3], const std::vector<int> &triangles_list)
{
	std::vector<diagnal> d_list;
	for (int i = 1; i < triangles_list.size(); i++)
	{
		const int *last_tri = triangles[triangles_list[i - 1]];
		const int *current_tri = triangles[triangles_list[i]];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			bool cond1 = current_tri[i] == last_tri[j] && current_tri[(i + 1) % 3] == last_tri[(j + 1) % 3];
			bool cond2 = current_tri[i] == last_tri[(j + 1) % 3] && current_tri[(i + 1) % 3] == last_tri[j];
			if (cond1 || cond2)
				d_list.push_back(std::make_pair(current_tri[i], current_tri[(i + 1) % 3]));
		}
	}
	return d_list;
}

std::vector<int> dfs(const int triangles[][3], int triangles_count, int start_tri, int end_tri)
{
	std::vector<int> triangles_list;
	bool *visited = new bool[triangles_count];
	int *parent = new int[triangles_count];
	memset(visited, false, triangles_count * sizeof(bool));
	memset(parent, -1, triangles_count * sizeof(int));

	std::stack<int> s;
	s.push(start_tri);
	while (!s.empty())
	{
		int v = s.top(); s.pop();
		if (!visited[v])
		{
			visited[v] = true;
			for (int u = 0; u < triangles_count; u++)
			if (neighbor_triangles(triangles, triangles_count, v, u) && !visited[u])
			{
				s.push(u);
				parent[u] = v;
			}
		}
		if (parent[end_tri] != -1)
			break;
	}
	int p = end_tri;
	while (parent[p] != -1)
	{
		triangles_list.push_back(p);
		p = parent[p];
	}
	triangles_list.push_back(start_tri);
	std::reverse(triangles_list.begin(), triangles_list.end());
	delete[] visited;
	delete[] parent;
	return triangles_list;
}

bool neighbor_triangles(const int triangles[][3], int triangles_count,
	int tri1, int tri2)
{
	if (tri1 != tri2)
	{
		const int *t1 = triangles[tri1];
		const int *t2 = triangles[tri2];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			bool cond1 = t1[i] == t2[j] && t1[(i + 1) % 3] == t2[(j + 1) % 3];
			bool cond2 = t1[i] == t2[(j + 1) % 3] && t1[(i + 1) % 3] == t2[j];
			if (cond1 || cond2)
				return true;
		}
	}
	return false;
}

int locate_point(const int triangles[][3], int triangles_count, const point &k)
{
	int i;
	for (i = 0; i < triangles_count; i++)
	{
		point p = polygon_points[triangles[i][0]];
		point q = polygon_points[triangles[i][1]];
		point r = polygon_points[triangles[i][2]];

		if (in_triangle(p, q, r, k))
			break;
	}
	return i;
}

bool in_triangle(const point &p, const point &q, const point &r, const point &k)
{
	bool pq_left = to_left(p, q, k);
	bool qr_left = to_left(q, r, k);
	bool rp_left = to_left(r, p, k);

	return (pq_left == qr_left) && (qr_left == rp_left);
}

bool to_left(const point &p, const point &q, const point &k)
{
	double px = p.x; double py = p.y;
	double qx = q.x; double qy = q.y;
	double kx = k.x; double ky = k.y;

	double area2 =
		px * qy - py * qx +
		qx * ky - qy * kx +
		kx * py - ky * px;
	return area2 >= 0;
}