/*
 * This file is part of the CGx-InverseK distribution (https://github.com/cgxeiji/CGx-InverseK).
 * Copyright (c) 2017 Eiji Onchi.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "math.h"
#include "InverseK.h"

_Inverse InverseK;

Link::Link() {
	_length = 0.0;
	_angleLow = 0.0;
	_angleHigh = 0.0;
	_angle = 0.0;
}

void Link::init(float length, float angle_low_limit, float angle_high_limit) {
	_length = length;
	_angleLow = angle_low_limit;
	_angleHigh = angle_high_limit;
}

bool Link::inRange(float angle) {
	return (angle >= _angleLow) && (angle <= _angleHigh);
}

float Link::getLength() {
	return _length;
}

float Link::getAngle() {
	return _angle;
}

void Link::setAngle(float angle) {
	_angle = angle;
}

// Initialize Inverse object
_Inverse::_Inverse() {
	_currentPhi = -DOUBLE_APPLE_PI;
}

// Load the links of the mechanism to be solved
void _Inverse::attach(Link shoulder, Link upperarm, Link forearm, Link hand) {
	_L0 = shoulder;
	_L1 = upperarm;
	_L2 = forearm;
	_L3 = hand;
}

// Cosine rule function
bool _Inverse::_cosrule(float opposite, float adjacent1, float adjacent2, float& angle) {
	float delta = 2 * adjacent1 * adjacent2;

	if (delta == 0) return false;

	float cos = (adjacent1*adjacent1 + adjacent2*adjacent2 - opposite*opposite) / delta;

	if ((cos > 1) || (cos < -1)) return false;

	angle = acos(cos);

	return true;
}

// Solve the angles for XY with a fixed attack angle
bool _Inverse::_solve(float x, float y, float phi, float& shoulder, float& elbow, float& wrist) {
	// Adjust coordinate system for base as ground plane
	float _r = sqrt(x*x + y*y);
	float _theta = atan2(y, x);
	float _x = _r * cos(_theta - HALF_APPLE_PI);
	float _y = _r * sin(_theta - HALF_APPLE_PI);
	float _phi = phi - HALF_APPLE_PI;

	// Find the coordinate for the wrist
	float xw = _x - _L3.getLength() * cos(_phi);
	float yw = _y - _L3.getLength() * sin(_phi);

	// Get polar system
	float alpha = atan2(yw, xw);
	float R = sqrt(xw*xw + yw*yw);

	// Calculate the inner angle of the shoulder
	float beta;
	if(!_cosrule(_L2.getLength(), R, _L1.getLength(), beta)) return false;

	// Calcula the inner angle of the elbow
	float gamma;
	if(!_cosrule(R, _L1.getLength(), _L2.getLength(), gamma)) return false;

	// Solve the angles of the arm
	float _shoulder, _elbow, _wrist;
	_shoulder = alpha - beta;
	_elbow = APPLE_PI - gamma;
	_wrist = _phi - _shoulder - _elbow;

	// Check the range of each hinge
	if (!_L1.inRange(_shoulder) || !_L2.inRange(_elbow) || !_L3.inRange(_wrist)) {
		// If not in range, solve for the second solution
		_shoulder += 2 * beta;
		_elbow *= -1;
		_wrist = _phi - _shoulder - _elbow;

		// Check the range for the second solution
		if (!_L1.inRange(_shoulder) || !_L2.inRange(_elbow) || !_L3.inRange(_wrist)) return false;
	}

	// Return the solution
	shoulder = _shoulder;
	elbow = _elbow;
	wrist = _wrist;

	return true;
}

// Solve the angles for XY with a free attack angle
bool _Inverse::_solve(float x, float y, float& shoulder, float& elbow, float& wrist) {
	if (_solve(x, y, _currentPhi, shoulder, elbow, wrist)) return true;
	for (float phi = -DOUBLE_APPLE_PI; phi < DOUBLE_APPLE_PI; phi += DEGREE_STEP) {
		if (_solve(x, y, phi, shoulder, elbow, wrist)) {
			_currentPhi = phi;
			return true;
		}
	}

	return false;
}

// Solve the angles for XYZ with a fixed attack angle
bool _Inverse::solve(float x, float y, float z, float& base, float& shoulder, float& elbow, float& wrist, float phi) {
	float hum_sq = _L1.getLength() * _L1.getLength();
	float uln_sq = _L2.getLength() * _L2.getLength();
	
	float wrist_init = wrist;
	float grip_angle_r = b2a(wrist);    //grip angle in radians for use in calculations
	  /* Base angle and radial distance from x,y coordinates */
	base = a2b(atan2(x, y));
	float rdist = sqrt((x * x) + (y * y));

	y = rdist;

	/* Grip offsets calculated based on grip angle */
	float grip_off_z = (sin(grip_angle_r)) * _L3.getLength();
	float grip_off_y = (cos(grip_angle_r)) * _L3.getLength();
	/* Wrist position */
	float wrist_z = (z - grip_off_z) - .08875;
	float wrist_y = y - grip_off_y;
	/* Shoulder to wrist distance ( AKA sw ) */
	float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
	float s_w_sqrt = sqrt(s_w);
	/* s_w angle to ground */
	//float a1 = atan2( wrist_y, wrist_z );
	float a1 = atan2(wrist_z, wrist_y);
	/* s_w angle to humerus */
	float d1 = ((hum_sq - uln_sq) + s_w) / (2.0 * _L1.getLength() * s_w_sqrt);
	float a2_p = -(fabs(fmod(d1, 2)) - 1.0);
	float a2 = acos(a2_p);
	/* shoulder angle */
	float shl_angle_r = a1 + a2;
	shoulder = a2b(shl_angle_r);
	/* elbow angle */
	float d = (hum_sq + uln_sq - s_w) / (2.0 * _L1.getLength() * _L2.getLength());
	float elb_angle_r_p = -(fabs(fmod(d, 2)) - 1.0);
	float elb_angle_r = acos(elb_angle_r_p);
	float elb_angle_d = a2b(elb_angle_r);
	elbow = -(180.0 - elb_angle_d);
	/* wrist angle */
	wrist = (wrist_init - elbow) - shoulder;

	return true;
}
