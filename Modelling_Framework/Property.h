#pragma once
class Property
{
private:
	virtual void init() = 0;
protected:
	int ID;
public:
	virtual double GetValue() = 0;
};

class Friction : public Property
{
private:
	double threshold;
	double mu_k, mu_v, n;
	double muk, sigma;
	void init() override;
public:
	Friction();
	Friction(int ID);
	Friction(int ID, double threshold, double mu_k, double mu_v, double n, double muk, double sigma);
	double GetValue() override;
	double GetValue(double force_normal, double velocity, double displacement);
	~Friction();
};