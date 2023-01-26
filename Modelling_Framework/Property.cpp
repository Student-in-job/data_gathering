#include "Property.h"

Friction::Friction()
{
	this->init();
};

Friction::~Friction() {};

Friction::Friction(int ID)
{
	this->init();
	this->ID = ID;
}

Friction::Friction(int ID, double threshold, double mu_k, double mu_v, double n, double muk, double sigma)
{
	this->ID = ID;
	this->threshold = threshold;
	this->mu_k = mu_k;
	this->mu_v = mu_v;
	this->n = n;
	this->muk = muk;
	this->sigma = sigma;
}

void Friction::init()
{
	this->ID = 0.0;
	this->threshold = 0.0;
	this->mu_k = 0.0;
	this->mu_v = 0.0;
	this->n = 0.0;
	this->muk = 0.0;
	this->sigma = 0.0;
}

double Friction::GetValue()
{
	return 0.0;
}

double Friction::GetValue(double force_normal, double velocity, double displacement)
{
	if (displacement > threshold)
		return mu_k * force_normal + mu_v * velocity + n;
	else
		return muk * force_normal + sigma * displacement;
}