#include "ofApp.h"

#include <glm/glm.hpp>
#include <glm/ext.hpp>

typedef glm::dvec2 Vec2;
typedef glm::dvec3 Vec3;
typedef glm::dvec4 Vec4;
typedef glm::dmat3 Mat3;
typedef glm::dmat4 Mat4;
typedef glm::dquat Quat;

struct PeseudoRandom {
	virtual ~PeseudoRandom() {}

	virtual uint32_t generate() = 0;
	virtual double uniform() = 0;
	virtual double uniform(double a, double b) = 0;
};
struct Xor : public PeseudoRandom {
	Xor() {

	}
	Xor(uint32_t seed) {
		_y = std::max(seed, 1u);
	}

	// 0 <= x <= 0x7FFFFFFF
	uint32_t generate() {
		_y = _y ^ (_y << 13); _y = _y ^ (_y >> 17);
		uint32_t value = _y = _y ^ (_y << 5); // 1 ~ 0xFFFFFFFF(4294967295
		return value >> 1;
	}
	// 0.0 <= x < 1.0
	double uniform() {
		return double(generate()) / double(0x80000000);
	}
	double uniform(double a, double b) {
		return a + (b - a) * double(uniform());
	}
public:
	uint32_t _y = 2463534242;
};

// Plane equation
// d = dot(n, p) for a given point p on the plane
struct Plane {
	Vec3 n;
	double d = 0.0;
};
inline Plane plane_from(const Vec3 &n, const Vec3 &p) {
	Plane plane;
	plane.n = n;
	plane.d = glm::dot(plane.n, p);
	return plane;
}

// 光線の後ろ側も有効なのに注意
// tminは見ない
inline bool intersect_ray_plane(const Vec3 &o, const Vec3 &d, const Plane &plane, double *tmin)
{
	double eps = 1.0e-9;
	double denom = glm::dot(plane.n, d);
	if (std::fabs(denom) < eps) {
		return false;
	}
	*tmin = (plane.d - glm::dot(plane.n, o)) / denom;
	return true;
}

inline Vec2 uniform_in_unit_circle(PeseudoRandom *random) {
	Vec2 d;
	double sq = 0.0;
	do {
		d.x = random->uniform(-1.0, 1.0);
		d.y = random->uniform(-1.0, 1.0);
		sq = glm::length2(d);
	} while (1.0 < sq);
	return d;
}

struct CameraSetting {
	double fovy = glm::radians(45.0);

	Vec3 eye = Vec3(0.0, 0.0, 1.0);
	Vec3 lookat = Vec3(0.0, 0.0, 0.0);
	Vec3 up = Vec3(0.0, 1.0, 0.0);

	int imageWidth = 4;
	int imageHeight = 3;

	double lensRadius = 0.1;
	double focalLength = 3.0;

	double tanThetaH() const {
		return std::tan(fovy * 0.5);
	}
	double distanceS() const {
		return imageHeight / (2.0 * tanThetaH());
	}
	double tanThetaV() const {
		return (double)imageWidth / distanceS();
	}

	double widthV() const {
		// 相似関係より
		return imageWidth * focalLength / distanceS();
	}
	double heightV() const {
		return imageHeight * focalLength / distanceS();
	}
};

class Camera {
public:
	Camera(const CameraSetting &setting):_setting(setting) {
		_view = glm::lookAt(_setting.eye, _setting.lookat, _setting.up);
		_viewInverse = glm::inverse(_view);

		Mat3 r(_viewInverse);
		_right = r[0];
		_up    = r[1];
		_back  = r[2];

		_planeV = plane_from(front(), origin() + front() * setting.focalLength);
	}

	Mat4 view() const {
		return _view;
	}
	Mat4 viewInverse() const {
		return _viewInverse;
	}
	Vec3 origin() const {
		return _viewInverse[3];
	}
	Vec3 front() const {
		return -_back;
	}
	Vec3 back() const {
		return _back;
	}

	Vec3 right() const {
		return _right;
	}
	Vec3 left() const {
		return -_right;
	}

	Vec3 up() const {
		return _up;
	}
	Vec3 down() const {
		return -_up;
	}

	CameraSetting setting() const {
		return _setting;
	}

	void sampleRay(PeseudoRandom *random, int x, int y, Vec3 *o, Vec3 *d) const {
		Vec2 sample = uniform_in_unit_circle(random);
		double r = setting().lensRadius;
		Vec3 sampleLens = origin() + r * right() * sample.x + r * down() * sample.y;
		
		auto focalPlaneCenter = origin() + front() * setting().focalLength;

		auto width = setting().widthV();
		auto height = setting().heightV();

		Vec3 LT = focalPlaneCenter
			+ left() * width * 0.5
			+ up() * height * 0.5;

		double stepPixel = width / setting().imageWidth;

		Vec3 PixelLT = LT + stepPixel * right() * (double)x + stepPixel * down() * (double)y;
		Vec3 sampleFocalPlane = PixelLT
			+ right() * random->uniform() * stepPixel
			+ down() * random->uniform() * stepPixel;

		*o = sampleLens;
		*d = glm::normalize(sampleFocalPlane - sampleLens);
	}
	double lensPDF() const {
		double r = _setting.lensRadius;
		return 1.0 / (glm::pi<double>() * r * r);
	}

	Plane planeV() const {
		return _planeV;
	}
	bool findPixel(const Vec3 &o, const Vec3 &d, int *x, int *y) const {
		// 逆向きの光線は無視
		if (0.0 < glm::dot(d, front())) {
			return false;
		}

		double tmin = std::numeric_limits<double>::max();
		if (intersect_ray_plane(o, d, planeV(), &tmin)) {
			Vec3 Vp = o + tmin * d;

			auto focalPlaneCenter = origin() + front() * setting().focalLength;
			auto width = setting().widthV();
			auto height = setting().heightV();

			Vec3 LT = focalPlaneCenter
				+ left() * width * 0.5
				+ up() * height * 0.5;

			Vec3 dir = Vp - LT;
			double Vx = glm::dot(dir, right());
			double Vy = glm::dot(dir, down());

			double stepPixel = width / setting().imageWidth;
			int at_x = (int)floor(Vx / stepPixel);
			int at_y = (int)floor(Vy / stepPixel);
			if (0 <= at_x && at_x < setting().imageWidth) {
				if (0 <= at_y && at_y < setting().imageHeight) {
					*x = at_x;
					*y = at_y;
					return true;
				}
			}
		}
		return false;
	}

	// 未テスト
	double Wi(const Vec3 &x1, const Vec3 &n1) const {
		Vec3 x1_to_x0 = x1 - origin();
		Vec3 x0_to_x1 = -x1_to_x0;
		// double G = glm::dot()
	}

	CameraSetting _setting;
	Mat4 _view;
	Mat4 _viewInverse;

	Vec3 _right;
	Vec3 _up;
	Vec3 _back;

	Plane _planeV;
};

inline ofVec3f toOf(Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}


//--------------------------------------------------------------
void ofApp::setup() {
	_camera.setNearClip(0.1f);
	_camera.setFarClip(1000.0f);
	_camera.setDistance(15.0f);

	ofSetFrameRate(60);
	ofSetVerticalSync(false);

	_imgui.setup();
}

//--------------------------------------------------------------
void ofApp::update() {

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofEnableDepthTest();

	static int sampleX = 0;
	static int sampleY = 0;

	static CameraSetting cameraSetting;
	Camera camera(cameraSetting);

	ofClear(0);
	_camera.begin();

	ofPushMatrix();
	ofRotateY(90.0f);
	ofSetColor(64);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofPushMatrix();
	ofDrawAxis(5);
	ofPopMatrix();

	ofSetColor(255);

	{
		ofPushMatrix();
		glm::mat4 m(camera.viewInverse());
		ofMultMatrix(glm::value_ptr(m));
		ofDrawAxis(1);
		ofPopMatrix();

		auto orig = camera.origin();
		ofDrawSphere(orig.x, orig.y, orig.z, 0.02);

		{
			// image plane
			auto imagePlaneCenter = orig + camera.front() * camera.setting().distanceS();
			ofLine(toOf(imagePlaneCenter), toOf(orig));

			double imageWidth = (double)camera.setting().imageWidth;
			double imageHeight = (double)camera.setting().imageHeight;
			Vec3 LT = imagePlaneCenter 
				+ camera.left() * imageWidth / 2.0
			    + camera.up() * imageHeight / 2.0;

			ofSetColor(128);
			for (int i = 0; i < camera.setting().imageHeight + 1; ++i) {
				ofDrawLine(
					toOf(LT + camera.down() * (double)i),
					toOf(LT + camera.down() * (double)i + camera.right() * imageWidth)
				);
			}
			for (int i = 0; i < camera.setting().imageWidth + 1; ++i) {
				ofDrawLine(
					toOf(LT + camera.right() * (double)i),
					toOf(LT + camera.right() * (double)i + camera.down() * imageHeight)
				);
			}
			ofSetColor(255);
			ofLine(toOf(orig), toOf(LT));
			ofLine(toOf(orig), toOf(LT + camera.right() * imageWidth));
			ofLine(toOf(orig), toOf(LT + camera.right() * imageWidth + camera.down() * imageHeight));
			ofLine(toOf(orig), toOf(LT + camera.down() * imageHeight));
		}

		{
			// focal plane
			auto focalPlaneCenter = orig + camera.front() * camera.setting().focalLength;

			auto width = camera.setting().widthV();
			auto height = camera.setting().heightV();

			Vec3 LT = focalPlaneCenter
				+ camera.left() * width / 2.0
				+ camera.up() * height / 2.0;

			Vec3 points[] = {
				LT,
				LT + camera.right() * width,
				LT + camera.right() * width + camera.down() * height,
				LT + camera.down() * height
			};
			for (int i = 0; i < 4; ++i) {
				ofDrawLine(toOf(points[i]), toOf(points[(i + 1) % 4]));
			}
		}

		// Sample
		{
			Xor random;

			bool insideSample =
				0 <= sampleX && sampleX < camera.setting().imageWidth &&
				0 <= sampleY && sampleY < camera.setting().imageHeight;

			for (int i = 0; i < 50; ++i) {
				Vec3 o;
				Vec3 d;
				camera.sampleRay(&random, sampleX, sampleY, &o, &d);

				ofSetColor(255);
				ofDrawLine(toOf(o), toOf(o + d * 10.0));

				double tmin = 1.0e+10;
				if (intersect_ray_plane(o, d, camera.planeV(), &tmin)) {
					Vec3 p = o + tmin * d;
					ofSetColor(255, 128, 0);
					ofDrawSphere(toOf(p), 0.02f);

					int x;
					int y;
					if (camera.findPixel(o, d, &x, &y)) {
						if (sampleX == x && sampleY == y) {
							// OK
						}
						else {
							abort();
						}
					}
				}
			}
		}

		double radius = camera.setting().lensRadius;
		ofPolyline line;
		int N = 20;
		double dTheta = glm::two_pi<double>() / N;
		for (int i = 0; i < N; ++i) {
			double dx = cos(dTheta * i);
			double dy = sin(dTheta * i);
			Vec3 arc = camera.origin() + radius * camera.right() * dx + radius * camera.up() * dy;
			line.addVertex(toOf(arc));
		}
		line.close();

		ofSetColor(255);
		line.draw();
	}

	_camera.end();

	_imgui.begin();
	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.0f, 0.2f, 0.2f, 0.5));
	ImGui::SetNextWindowPos(ofVec2f(10, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, ofGetHeight() * 0.8), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::InputInt("imageWidth", &cameraSetting.imageWidth);
	ImGui::InputInt("imageHeight", &cameraSetting.imageHeight);
	{
		glm::vec3 eye = cameraSetting.eye;
		ImGui::InputFloat("eye x", &eye.x, 0.1f);
		ImGui::InputFloat("eye y", &eye.y, 0.1f);
		ImGui::InputFloat("eye z", &eye.z, 0.1f);
		cameraSetting.eye = eye;
	}
	{
		glm::vec3 lookat = cameraSetting.lookat;
		ImGui::InputFloat("lookat x", &lookat.x, 0.1f);
		ImGui::InputFloat("lookat y", &lookat.y, 0.1f);
		ImGui::InputFloat("lookat z", &lookat.z, 0.1f);
		cameraSetting.lookat = lookat;
	}

	{
		float fovy = cameraSetting.fovy;
		ImGui::Text("fovy degree: %.2f", glm::degrees(fovy));
		ImGui::InputFloat("fovy", &fovy, 0.05f);
		cameraSetting.fovy = fovy;
	}
	{
		float focalLength = cameraSetting.focalLength;
		ImGui::InputFloat("focalLength", &focalLength, 0.1f);
		cameraSetting.focalLength = focalLength;
	}
	{
		float lensRadius = cameraSetting.lensRadius;
		ImGui::InputFloat("lensRadius", &lensRadius, 0.1f);
		cameraSetting.lensRadius = lensRadius;
	}

	ImGui::Separator();
	ImGui::InputInt("sampleX", &sampleX);
	ImGui::InputInt("sampleY", &sampleY);

	static bool randomSample = false;
	ImGui::Checkbox("Random Sample", &randomSample);
	if (randomSample) {
		static Xor random;
		sampleX = (int)random.uniform(-2, camera.setting().imageWidth + 2);
		sampleY = (int)random.uniform(-2, camera.setting().imageHeight + 2);
	}

	ImGui::End();
	ImGui::PopStyleColor();

	_imgui.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
