#include <cstdio>
#include <cmath>
#include <vector>
#include <fstream>
#include <thread>
#include <string>
#include <atomic>

template <typename T>
struct Point {
	T X;
	T Y;
	T Z;
	
	Point<T> operator+(const Point<T>& rhs) const
	{
		return Point<T>{ X: this->X+rhs.X, Y: this->Y+rhs.Y, Z: this->Z+rhs.Z };
	}
	
	Point<T> operator-(const Point<T>& rhs) const
	{
		return Point<T>{ X: this->X-rhs.X, Y: this->Y-rhs.Y, Z: this->Z-rhs.Z };
	}
	
	Point<T> operator*(const double rhs) const
	{
		return Point<T>{ X: this->X*rhs, Y: this->Y*rhs, Z: this->Z*rhs };
	}
	
	Point<T> operator*(const Point<T>& rhs) const
	{
		return Point<T>{
			X: this->Y*rhs.Z - this->Z*rhs.Y,
			Y: this->Z*rhs.X - this->X*rhs.Z,
			Z: this->X*rhs.Y - this->Y*rhs.X };
	}
	
	double magnitude() const
	{
		return sqrt(X*X+Y*Y+Z*Z);
	}
	
	double magnitudeSq() const
	{
		return X*X+Y*Y+Z*Z;
	}
	
	Point<T>& normalize()
	{
		double invMagn = 1.0 / magnitude();
		X *= invMagn;
		Y *= invMagn;
		Z *= invMagn;
		
		return *this;
	}
	
	Point<T> normalize() const
	{
		Point<T> lCopy{X:this->X, Y:this->Y, Z:this->Z};
		lCopy.normalize();
		return lCopy;
	}
	
	Point<T>& normalize(const double pMagnitude)
	{
		operator *(pMagnitude / magnitude());
		return *this;
	}
	
	Point<T> normalize(const double pMagnitude) const
	{
		Point<T> lValue{*this};
		lValue.normalize(pMagnitude);
		return lValue;
	}
};

template <typename T>
struct Size {
	T W;
	T H;
};

template <typename T>
struct Color {
	T R;
	T G;
	T B;
};

template <typename T>
struct Rect {
	T X;
	T Y;
	T W;
	T H;
};

// références pour chaque dimension : permet de modifier la plage occupée pour une valeur
// donnée de manière à ce que ça rentre dans un long double.
constexpr long double operator "" _refMass_kg(long double d)	{ return 1.0; }
constexpr long double operator "" _refDistance_m(long double d)	{ return 1.0; }
constexpr long double operator "" _refTime_s(long double d)		{ return 1.0; }

// distance
constexpr long double operator "" _m(long double d)		{ return d / 1.0_refDistance_m; }
constexpr long double operator "" _ly(long double d)	{ return d * 9.461E15_m; }
constexpr long double operator "" _au(long double d)	{ return d * 149597870700.0_m; }
constexpr long double operator "" _km(long double d)	{ return d * 1000.0_m; }
constexpr long double operator "" _cm(long double d)	{ return d * 0.01_m; }
constexpr long double operator "" _mm(long double d)	{ return d * 0.001_m; }
constexpr long double operator "" _kg(long double d)	{ return d / 1.0_refMass_kg; }

constexpr long double operator "" _m2(long double d)	{ return d * (1.0_m * 1.0_m); }
constexpr long double operator "" _m3(long double d)	{ return d * (1.0_m * 1.0_m * 1.0_m); }

// temps
constexpr long double operator "" _s(long double d)		{ return d / 1.0_refTime_s; }
constexpr long double operator "" _ms(long double d)	{ return d * 0.001_s; }

constexpr long double operator "" _s2(long double d)	{ return d * (1.0_s * 1.0_s); }
constexpr long double operator "" _s3(long double d)	{ return d * (1.0_s * 1.0_s * 1.0_s); }

// vitesse
constexpr long double operator "" _m_s(long double d)	{ return d / (1.0_m / 1.0_s); }
constexpr long double operator "" _c(long double d)		{ return d * 3.0E8_m_s; }

// autres
constexpr long double operator "" _m3_kg1s2(long double d)	{ return d / (1.0_m3 / (1.0_kg * 1.0_s2)); } // m3.kg-1.s-2

template <typename T> using Vector = Point<T>;
typedef Point<double> Pointd;
typedef Vector<double> Vectord;
typedef Size<int> Sizei;
typedef Size<double> Sized;
typedef Color<unsigned char> Colorb;
typedef Rect<int> Recti;

struct Body {
	Pointd Position;
	double Radius;
	double Mass;
	Colorb Diffuse;
};

Sizei gScreenSize { W: 320, H: 240 };
Sized gSensorSize { W: 4.0_cm, H: 3.0_cm };
double gFocalLength { 4.0_cm };

Body gSun {
	Position: {
		X: 0.0_ly,
		Y: 0.0_ly,
		Z: 0.0_ly },
	Radius: 696E3_km,
	Mass: 1.9891E30_kg,
	Diffuse: { R: 255, G: 255, B: 0 } };
	
Body gEarth {
	Position: {
		X: -0.1_au,
		Y:  0.0_au,
		Z:  0.0_au },
	Radius: 6.35E3_km,
	Mass: 5.9736E24_kg * 1E9,
	Diffuse: { R: 0, G: 100, B: 255 } };
	
std::vector<Body*> gBodies{&gSun, &gEarth};
	
Pointd gObserver {
	X: -0.101_au, 
	Y:  0.0_au,
	Z:  0.0_au };
	
Vectord gObserverLook {
	X: 1.0,
	Y: 0.0,
	Z: 0.0 };
	
Vectord gObserverUp {
	X: 0.0,
	Y: 0.0,
	Z: 1.0 };

Vectord getPixelPhotonDirection(int pX, int pY)
{
	Vectord lSensorCenter = gObserver + gObserverLook * gFocalLength;
	Vectord lSensorHeight = gObserverUp * gSensorSize.H;
	Vectord lSensorWidth = (gObserverLook * gObserverUp) * gSensorSize.W;
	Vectord lSensorOrigin = lSensorCenter - lSensorWidth*0.5 - lSensorHeight*0.5;
	Vectord lPixelCenter = lSensorOrigin +
		lSensorWidth  * ((double)pX / (double)gScreenSize.W) +
		lSensorHeight * ((double)pY / (double)gScreenSize.H);
	
	Vectord lPhotonDir = lPixelCenter - gObserver;
	lPhotonDir.normalize();
	return lPhotonDir;
}

bool isInsideBody(const Pointd& pPoint, const Body& pBody)
{
	return ((pPoint - pBody.Position).magnitudeSq() <= pBody.Radius*pBody.Radius);
}

Colorb getBodyColor(const Body& pBody, const Pointd& pPoint)
{
	return pBody.Diffuse;
}

Vectord getPhotonPseudoAcceleration(const Pointd& pPosition, const Body& pBody)
{
	const double clGravConst = 6.67384E-11_m3_kg1s2;
	
	Vectord lDirection = pBody.Position - pPosition;
	double lDistanceSq = lDirection.magnitudeSq();
	double g = clGravConst * pBody.Mass / lDistanceSq;
	
	lDirection = lDirection.normalize() * g;
	
	return lDirection;
}

double getTimeStep(const Pointd& pPoint)
{
	// Renvoie le pas de calcul à utiliser en fonction de la proximité du photon avec les différents
	// corps célestes.
	// Plus on est éloigné des corps, plus on peut se permettre d'avoir un pas important.
	
	const double clDistanceFactor = 1.0; // plus c'est grand, plus le delta de temps sera grand; doit être <= 1.0
	double lMinRefDistance = std::numeric_limits<double>::max();
	for(const Body* lBody : gBodies)
	{
		double lRefDistance = clDistanceFactor * (pPoint - lBody->Position).magnitude();
		if (lRefDistance < lMinRefDistance)
			lMinRefDistance = lRefDistance;
	}
	
	// On retourne le temps mis par la lumière pour parcourir la distance de référence sélectionnée
	return (lMinRefDistance / 1.0_c);
}

void travelPhoton(const Pointd& pOrigin, const Vectord& pInitialDir, const Body*& pCollider, Pointd& lCollision)
{
	pCollider = nullptr;
	
    const double clMaxTravel = 1.5_au;
    const double clMaxPhotonLife = clMaxTravel / 1.0_c;
    
    double lPhotonLife = 0.0;
    
    Pointd lPosition = pOrigin;
    Vectord lVelocity = pInitialDir.normalize() * 1.0_c;

    while(lPhotonLife < clMaxPhotonLife)
    {   
		// Détermination du pas de calcul
		double lTimeStep = getTimeStep(lPosition);
		lPhotonLife += lTimeStep;
		
		// Calcul de la pseudo accélération provoquant la déviation du photon     
        Vectord lPseudoAcceleration{X:0.0, Y:0.0, Z:0.0};
        for(const Body* lBody : gBodies)
			lPseudoAcceleration = lPseudoAcceleration + getPhotonPseudoAcceleration(lPosition, *lBody);
			
		// Mise à jour de la direction du photon
		lVelocity = lVelocity + lPseudoAcceleration * lTimeStep;
		lVelocity.normalize(1.0_c);
		
		// Mise à jour de la position du photon
		lPosition = lPosition + lVelocity * lTimeStep;

        // Détermination des intersections
        for(const Body* lBody : gBodies)
        {
			if (isInsideBody(lPosition, *lBody) == true)
			{
				pCollider = lBody;
				lCollision = lPosition;
				return;
			}
		}
    }
}

Colorb getPixelColor(int pX, int pY)
{
	Vectord lPhotonDirection = getPixelPhotonDirection(pX, pY);
	const Body* lCollider = nullptr;
	Pointd lCollision;
	travelPhoton(gObserver, lPhotonDirection, lCollider, lCollision);
	
	if (lCollider == nullptr)
		return Colorb{ R: 0, G: 0, B: 0 };
	else
	{
		Colorb lColor = getBodyColor(*lCollider, lCollision);
		return lColor;
	}
}

void renderImagePart(Recti pPart, std::reference_wrapper<std::vector<Colorb>> pOut, std::reference_wrapper<std::atomic<int>> pCounter)
{
	std::vector<Colorb>& lOut = pOut.get();
	for(int lY = pPart.Y; lY < pPart.Y+pPart.H; ++lY)
	{
		for(int lX = pPart.X; lX < pPart.X+pPart.W; ++lX)
		{
			int lCounter = (pCounter.get() += 1);
			double lPercent = 100.0 * (double)lCounter / (double)(gScreenSize.W * gScreenSize.H);
			
			if (lCounter % 5000 == 0)
				printf("Rendering pixel #%010d (%f %%)\n", lCounter, lPercent);
			
			Colorb lColor = getPixelColor(lX, lY);
			lOut.at(lY * gScreenSize.W + lX) = lColor;
		}
	}
}

void renderImage(const std::string& pFilename)
{
	// Initialisation
	std::atomic<int> lCounter{0};
	std::vector<Colorb> lPixels;
	lPixels.resize(gScreenSize.W * gScreenSize.H);
	
	// Lancement des calculs
	std::vector<std::thread> lThreads;
	const int clXCount = 2;
	const int clYCount = 2;
	for(int lX = 0; lX < clXCount; ++lX)
	{
		for(int lY = 0; lY < clYCount; ++lY)
		{
			Recti lPart{
				X: lX * gScreenSize.W / clXCount,
				Y: lY * gScreenSize.H / clYCount,
				W: gScreenSize.W / clXCount,
				H: gScreenSize.H / clYCount };
				
			std::thread lThread(
				&renderImagePart,
				lPart,
				std::ref(lPixels),
				std::ref(lCounter));
				
			lThreads.push_back(std::move(lThread));
		}
	}
	
	// Attente de la fin des calculs
	for(auto& lThread : lThreads)
		lThread.join();

	// Enregistrement du résultat
	std::fstream lFile(pFilename, std::ios::out);	
	for(const Colorb& lPixel : lPixels)
	{
		lFile.write((char*)&lPixel.R, sizeof(lPixel.R));
		lFile.write((char*)&lPixel.G, sizeof(lPixel.G));
		lFile.write((char*)&lPixel.B, sizeof(lPixel.B));
	}
}

int main(int argc, char** argv)
{
	char tmp[255];
	for(int i = 0; i < 200; ++i)
	{
		gObserver.Y = (double)(i-100) * 0.00001_au;
		sprintf(tmp, "output/image_%04d.data", i);
		renderImage(std::string(tmp));
	}
	return 0;
}

/*
int main_old(int argc, char** argv)
{
    const double c = 3.0E8; // m.s-1
    const double c2 = c*c;
    double M = 1E35; // kg
    double X = - 30.0 * c; // 10 secondes lumière, m
    double Y =   1.0 * c; // 1 seconde lumière, m
    double VX = c;
    double VY = 0.0;
    
    const double GravConst = 6.67384E-11;

    const double dt = 1E-3 ; // pas de calcul

    double DistSq = X*X + Y*Y;
    double g_org = GravConst * M / DistSq;

    while (true)
    {
        DistSq = X*X + Y*Y;
        double InvDist = 1.0 / sqrt(DistSq);
        double g = GravConst * M / DistSq;
        double DirX = -X * InvDist;
        double DirY = -Y * InvDist;
        double dVX = g * dt * DirX;
        double dVY = g * dt * DirY;
        VX += dVX;
        VY += dVY;
        
        double VSq = VX*VX + VY*VY;
        double cInvV = c / sqrt(VSq);
        VX *= cInvV;
        VY *= cInvV;

        X += VX * dt;
        Y += VY * dt;

        double FreqCoeff = 1.0 - (g - g_org) / c2;

        printf("X = %f, Y = %f, VX = %f, VY = %f, fr = %f\n", X, Y, VX, VY, FreqCoeff);
    }

	return 0;
}
*/
