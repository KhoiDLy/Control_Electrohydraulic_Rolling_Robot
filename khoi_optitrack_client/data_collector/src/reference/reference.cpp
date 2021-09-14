#include "reference/reference.hpp"

Constant::Constant(float speed) {
	this->v_ref = speed;
}

Constant::Constant() {
	this->v_ref = 0.0;
}

SineWave::SineWave(float wavelength, float amplitude) {
	this->wavelength = wavelength;
	this->amplitude = amplitude;
}

Stairs::Stairs(std::vector<float> speeds, std::vector<int> intervals) {
	this->speeds = speeds;
	this->intervals = intervals;
	int cumsum = 0;
	if (speeds.size() != intervals.size()) {
		std::cerr << "speeds and intervals must be same length" << std::endl;
	}
	else {
		for (int i = 0; i < speeds.size(); i++) {
			cumsum += intervals[i];
			this->ordered_map.insert(std::pair<int, float>(cumsum, speeds[i]));
			this->total_duration += intervals[i];
		}
	}
}

Stairs::Stairs() {
	this->speeds = { 2., 2., 2., };
	this->intervals = { 100, 1000, 10000 };
	this->total_duration = 0;
	for (int i = 0; i < speeds.size(); i++) {
		this->ordered_map.insert(std::pair<int, float>(this->total_duration, speeds[i]));
		this->total_duration += intervals[i];
	}
}

Ramp::Ramp(float slope, int interval, float v0) {
	this->slope = slope;
	this->interval = interval;
	this->v0 = v0;
}

Ramp::Ramp() {
	this->slope = 2.;
	this->interval = 100000;
	this->v0 = 0.0;
}

KhoisPattern::KhoisPattern(Stairs initial_stairs, Ramp first_ramp, Ramp second_ramp, Constant final) {
	this->stairs = initial_stairs;
	this->ramp1 = first_ramp;
	this->ramp2 = second_ramp;
	this->final = final;
}

std::vector<float> Constant::spin_reference(int timestep, int t0, int pred_hor) {
	std::vector<float> reference_speed;
	for (int i = 0; i < pred_hor; i++) {
		reference_speed.push_back(this->v_ref);
	}
	return reference_speed;
}

std::vector<float> SineWave::spin_reference(int timestep, int t0, int pred_hor) {
	std::vector<float> reference_speed;
	for (int i = 0; i < pred_hor; i++) {
		reference_speed.push_back(this->amplitude * sin((timestep - t0 + i) / this->wavelength));
	}
	return reference_speed;
}

std::vector<float> Stairs::spin_reference(int timestep, int t0, int pred_hor) {
	std::map<int, float>::iterator it = this->ordered_map.begin();
	std::vector<float> reference_speeds;
	
	for (int i = 0; i < pred_hor; i++) {
		int current_timestep = timestep + i - t0;
		
		while (current_timestep > it->first && it!=this->ordered_map.end()) {
			it++;
		} 

		if (it == this->ordered_map.end()) {
			reference_speeds.push_back(0.0);
		}
		else {
			reference_speeds.push_back(it->second);
		}
	}
	return reference_speeds;
}

std::vector<float> Ramp::spin_reference(int timestep, int t0, int pred_hor) {
	std::vector<float> reference_speeds;
	for (int i = 0; i < pred_hor; i++) {
		int current_timestep = timestep + i - t0;
		reference_speeds.push_back(this->slope * current_timestep + this->v0);
	}
	return reference_speeds;
}

std::vector<float> KhoisPattern::spin_reference(int timestep, int t0, int pred_hor) {
	std::vector<float> reference_speeds;
	int time_0 = this->stairs.total_duration;
	int time_1 = time_0 + this->ramp1.interval;
	int time_2 = time_1 + this->ramp2.interval;
	
	for (int i = 0; i < pred_hor; i++) {
		int current_timestep = timestep + i - t0;
		std::vector<float> current_speed;
		if (current_timestep < time_0) {
			current_speed = this->stairs.spin_reference(timestep, t0, 1);
			reference_speeds.push_back(current_speed[0]);
		}
		else if (current_timestep >= time_0 && current_timestep < time_1) {
			current_speed = this->ramp1.spin_reference(timestep, t0 + time_0, 1);
			reference_speeds.push_back(current_speed[0]);
		}
		else if (current_timestep >= time_1 && current_timestep < time_2) {
			
			current_speed = this->ramp2.spin_reference(timestep, t0 + time_1, 1);
			reference_speeds.push_back(current_speed[0]);
		}
		else if (current_timestep >= time_2) {
			current_speed = this->final.spin_reference(timestep , t0 + time_2, 1);
			reference_speeds.push_back(current_speed[0]);
		}
	}
	return reference_speeds;
}


