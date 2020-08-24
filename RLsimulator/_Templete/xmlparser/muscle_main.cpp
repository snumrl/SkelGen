#include <tinyxml.h>
#include <Eigen/Core>
#include <map>
#include <vector>
#include <fstream>
#define LARGE_VALUE 1E6
#define REVOLUTE_JOINT_LIMIT 0.05
#define PROXIMAR_JOINT_LIMIT 1.0
namespace Eigen
{
	typedef Matrix<double,1,1> Vector1d;
};
struct UserConstant
{
	UserConstant(double _f0,double _lm,double _lt,double angle,double lmax)
		:f0(_f0),lm(_lm),lt(_lt),pen_angle(angle),l_max(lmax)
	{}
	double f0;
	double lm;
	double lt;
	double pen_angle;
	double l_max;	
};
struct MayaConstant
{
	MayaConstant(std::string name)
		:mName(name)
	{}
	void AddAnchor(std::string body,Eigen::Vector3d glob_pos)
	{
		mAnchors.push_back(std::make_pair(body,glob_pos));
	}
	std::string mName;
	std::vector<std::pair<std::string,Eigen::Vector3d>> mAnchors;
};
std::string toString(const Eigen::Vector3d& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;

}
std::string toString(const Eigen::VectorXd& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;	
}
void ReadMayaConstant(std::vector<MayaConstant>& mc,std::string path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	std::string name;
	double x,y,z;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;
		if(index=="unit")
		{
			ss>>name;
			// std::cout<<"unit "<<name<<std::endl;
			// std::cout<<name<<std::endl;
			mc.push_back(MayaConstant(name));
		}
		else if(index=="attach")
		{
			ss>>name;

			ss>>x>>y>>z;
			// std::cout<<"attach "<<name<<" "<<-x<<" "<<y<<" "<<z<<std::endl;
			mc.back().AddAnchor(name,Eigen::Vector3d(x,y,z));
		}
	}

	ifs.close();
}
int main(int argc,char** argv)
{
	std::map<std::string,UserConstant> ucs;

	std::vector<MayaConstant> mcs;
	ReadMayaConstant(mcs,argv[1]);
	// exit(0);
	// std::vector<std::string> knee_flexor;
	// knee_flexor.push_back("L_Bicep_Femoris_Short");
	// knee_flexor.push_back("L_Gastrocnemius_Lateral_Head");
	// knee_flexor.push_back("L_Gastrocnemius_Medial_Head");
	// knee_flexor.push_back("L_Gracilis");
	// knee_flexor.push_back("L_Popliteus");
	// knee_flexor.push_back("L_Sartorius");
	// std::vector<std::string> knee_extensor;
	// knee_extensor.push_back("L_Bicep_Femoris_Longus");
	// knee_extensor.push_back("L_Rectus_Femoris");
	// knee_extensor.push_back("L_Semimembranosus1");
	// knee_extensor.push_back("L_Semitendinosus");
	// knee_extensor.push_back("L_Tensor_Fascia_Lata2");
	// knee_extensor.push_back("L_Vastus_Intermedius1");
	// knee_extensor.push_back("L_Vastus_Lateralis1");
	// knee_extensor.push_back("L_Vastus_Medialis2");							
	ucs.insert(std::make_pair("L_Adductor_Brevis",UserConstant(303.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Brevis1",UserConstant(303.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Longus",UserConstant(399.5/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Longus1",UserConstant(399.5/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Magnus",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Magnus1",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Magnus2",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Magnus3",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Adductor_Magnus4",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Bicep_Femoris_Longus",UserConstant(705.2,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Bicep_Femoris_Short",UserConstant(315.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Bicep_Femoris_Short1",UserConstant(315.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus1",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus2",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus3",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Extensor_Hallucis_Longus",UserConstant(165.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus1",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus2",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus3",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Hallucis",UserConstant(436.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Hallucis1",UserConstant(436.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gastrocnemius_Lateral_Head",UserConstant(606.4,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gastrocnemius_Medial_Head",UserConstant(1308.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Maximus",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Maximus1",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Maximus2",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Maximus3",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Maximus4",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Medius",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Medius1",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Medius2",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Medius3",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Minimus",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Minimus1",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gluteus_Minimus2",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Gracilis",UserConstant(137.3,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Pectineus",UserConstant(177.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Peroneus_Brevis",UserConstant(305.9,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Peroneus_Longus",UserConstant(653.3,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Peroneus_Tertius",UserConstant(90.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Peroneus_Tertius1",UserConstant(90.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Piriformis",UserConstant(296.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Piriformis1",UserConstant(296.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Psoas_Major",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Psoas_Major1",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Psoas_Major2",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Psoas_Minor",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Quadratus_Femoris",UserConstant(254.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Rectus_Femoris",UserConstant(848.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Rectus_Femoris1",UserConstant(848.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Sartorius",UserConstant(113.5,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Semimembranosus",UserConstant(1162.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Semimembranosus1",UserConstant(1162.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Semitendinosus",UserConstant(301.9,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Soleus",UserConstant(3585.9/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Soleus1",UserConstant(3585.9/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Tensor_Fascia_Lata",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Tensor_Fascia_Lata1",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Tensor_Fascia_Lata2",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Tibialis_Anterior",UserConstant(673.7,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Tibialis_Posterior",UserConstant(905.6,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Intermedius",UserConstant(1024.2/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Intermedius1",UserConstant(1024.2/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Lateralis",UserConstant(2255.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Lateralis1",UserConstant(2255.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Medialis",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Medialis1",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Vastus_Medialis2",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_iliacus",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_iliacus1",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_iliacus2",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Flexor_Digiti_Minimi_Brevis_Foot",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Inferior_Gemellus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Obturator_Externus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Obturator_Internus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Plantaris",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Popliteus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("L_Superior_Gemellus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));


	ucs.insert(std::make_pair("R_Adductor_Brevis",UserConstant(303.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Brevis1",UserConstant(303.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Longus",UserConstant(399.5/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Longus1",UserConstant(399.5/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Magnus",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Magnus1",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Magnus2",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Magnus3",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Adductor_Magnus4",UserConstant(1296.9/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Bicep_Femoris_Longus",UserConstant(705.2,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Bicep_Femoris_Short",UserConstant(315.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Bicep_Femoris_Short1",UserConstant(315.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Extensor_Digitorum_Longus",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Extensor_Digitorum_Longus1",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Extensor_Digitorum_Longus2",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Extensor_Digitorum_Longus3",UserConstant(345.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Extensor_Hallucis_Longus",UserConstant(165.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Digitorum_Longus",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Digitorum_Longus1",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Digitorum_Longus2",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Digitorum_Longus3",UserConstant(274.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Hallucis",UserConstant(436.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Hallucis1",UserConstant(436.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gastrocnemius_Lateral_Head",UserConstant(606.4,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gastrocnemius_Medial_Head",UserConstant(1308.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Maximus",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Maximus1",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Maximus2",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Maximus3",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Maximus4",UserConstant(1852.6/5.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Medius",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Medius1",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Medius2",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Medius3",UserConstant(2199.6/4.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Minimus",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Minimus1",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gluteus_Minimus2",UserConstant(595.0/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Gracilis",UserConstant(137.3,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Pectineus",UserConstant(177.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Peroneus_Brevis",UserConstant(305.9,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Peroneus_Longus",UserConstant(653.3,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Peroneus_Tertius",UserConstant(90.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Peroneus_Tertius1",UserConstant(90.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Piriformis",UserConstant(296.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Piriformis1",UserConstant(296.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Psoas_Major",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Psoas_Major1",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Psoas_Major2",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Psoas_Minor",UserConstant(479.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Quadratus_Femoris",UserConstant(254.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Rectus_Femoris",UserConstant(848.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Rectus_Femoris1",UserConstant(848.8/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Sartorius",UserConstant(113.5,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Semimembranosus",UserConstant(1162.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Semimembranosus1",UserConstant(1162.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Semitendinosus",UserConstant(301.9,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Soleus",UserConstant(3585.9/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Soleus1",UserConstant(3585.9/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Tensor_Fascia_Lata",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Tensor_Fascia_Lata1",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Tensor_Fascia_Lata2",UserConstant(155.0/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Tibialis_Anterior",UserConstant(673.7,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Tibialis_Posterior",UserConstant(905.6,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Intermedius",UserConstant(1024.2/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Intermedius1",UserConstant(1024.2/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Lateralis",UserConstant(2255.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Lateralis1",UserConstant(2255.4/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Medialis",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Medialis1",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Vastus_Medialis2",UserConstant(1443.7/2.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_iliacus",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_iliacus1",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_iliacus2",UserConstant(621.9/3.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Flexor_Digiti_Minimi_Brevis_Foot",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Inferior_Gemellus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Obturator_Externus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Obturator_Internus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Plantaris",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Popliteus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	ucs.insert(std::make_pair("R_Superior_Gemellus",UserConstant(50.0,1.0,0.2,0.0,-0.1)));
	for(int i =0;i<mcs.size();i++){
		if(ucs.find(mcs[i].mName) == ucs.end())
			ucs.insert(std::make_pair(mcs[i].mName,UserConstant(1000.0,1.1,0.2,0.0,-0.1)));
	}
	// ucs.at("L_Rectus_Femoris").l_max = 1.15;
	// ucs.at("L_Rectus_Femoris").l_max = 1.0-0.02;
	// ucs.at("L_Vastus_Intermedius1").l_max = 1.0;
	// ucs.at("R_Vastus_Intermedius1").l_max = 1.0;
	// ucs.at("R_Rectus_Femoris").l_max = 1.0-0.02;

	// ucs.at("L_Gluteus_Maximus3").l_max = 1.0;
	// ucs.at("L_Soleus1").l_max = 0.95;
	ucs.at("R_Soleus1").l_max = 1.95;
	// ucs.at("L_Extensor_Digitorum_Longus2").l_max =0.975;
	// ucs.at("L_Extensor_Hallucis_Longus").l_max =0.975;

	// ucs.at("R_Soleus1").l_max = 0.95;
	// ucs.at("R_Extensor_Digitorum_Longus2").l_max =0.975;
	// ucs.at("R_Extensor_Hallucis_Longus").l_max =0.975;

	// // For tibia

	// ucs.at("L_Semimembranosus").l_max = 1.0;
	// ucs.at("L_Semitendinosus").l_max = 1.0-0.027;
	

	// ucs.at("R_Semitendinosus").l_max = 1.0-0.03;
	// ucs.at("R_Vastus_Intermedius1").l_max = 1.03;
	// // ucs.at("L_Adductor_Magnus2").l_max = 1.2;

	// ucs.at("R_Semitendinosus").l_max = 1.02;
	// ucs.at("R_Vastus_Intermedius1").l_max = 1.06;
	// // ucs.at("R_Adductor_Magnus2").l_max = 1.2;
	// ucs.at("L_Psoas_Major1").l_max = 0.96;
	// ucs.at("L_Sartorius").l_max = 0.9;
	// ucs.at("L_Tensor_Fascia_Lata2").l_max = 0.98;

	// ucs.at("L_Psoas_Major1").l_max = 0.96;
	// ucs.at("R_Psoas_Major1").l_max = 0.97+0.02;
	// ucs.at("L_Psoas_Major1").l_max = 0.97+0.02;
	// ucs.at("L_iliacus1").l_max = 0.98;
	// ucs.at("R_iliacus1").l_max = 0.98;
	// ucs.at("R_Sartorius").l_max = 0.9;
	// ucs.at("R_Tensor_Fascia_Lata2").l_max = 0.98+0.02;
	// ucs.at("L_Tensor_Fascia_Lata2").l_max = 0.98+0.02;
// ucs.at("L_Bicep_Femoris_Longus").l_max = 1.0;
// ucs.at("R_Bicep_Femoris_Longus").l_max = 1.0;
// ucs.at("L_Gastrocnemius_Lateral_Head").l_max = 0.95;
// ucs.at("L_Semimembranosus1").l_max = 0.95;
// ucs.at("R_Semimembranosus1").l_max = 0.95;
// ucs.at("L_Semitendinosus").l_max = 1.2;
	// for(int i=0;i<knee_flexor.size();i++)
	// {
	// 	ucs.at(knee_flexor[i]).lm = 0.4;
	// 	ucs.at(knee_flexor[i]).lt = 0.4;
	// }
	// for(int i=0;i<knee_extensor.size();i++)
	// {
	// 	ucs.at(knee_extensor[i]).lm = 0.7;
	// 	ucs.at(knee_extensor[i]).lt = 0.2;
	// }

	// ucs.insert(std::make_pair("L_Adductor_Longus1",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Bicep_Femoris_Longus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Bicep_Femoris_Short",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Extensor_Hallucis_Longus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus2",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Flexor_Hallucis",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gastrocnemius_Lateral_Head",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus2",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus4",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Medius1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gracilis",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Obturator_Externus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Obturator_Internus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Peroneus_Longus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Plantaris",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Psoas_Major2",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Rectus_Femoris",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Sartorius",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Semimembranosus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Semitendinosus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Soleus1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Tensor_Fascia_Lata2",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Tibialis_Anterior",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Intermedius1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Lateralis",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Medialis1",UserConstant(500.0,1.0,0.2,0.0)));
	std::vector<std::string> upper_body;
	upper_body.push_back("TibiaL");
	upper_body.push_back("TalusL");
	upper_body.push_back("FootThumbL");
	upper_body.push_back("FootPinkyL");
	// upper_body.push_back("Pevlis");
	// upper_body.push_back("Spine");
	// upper_body.push_back("Torso");
	// upper_body.push_back("Neck");
	// upper_body.push_back("Head");
	// upper_body.push_back("ShoulderL");
	// upper_body.push_back("ArmL");
	// upper_body.push_back("ForeArmL");
	// upper_body.push_back("HandL");
	// upper_body.push_back("ShoulderR");
	// upper_body.push_back("ArmR");
	// upper_body.push_back("ForeArmR");
	// upper_body.push_back("HandR");
	// upper_body.push_back("FemurR");
	// upper_body.push_back("TibiaR");
	// upper_body.push_back("TalusR");


	// upper_body.push_back("FemurL");
	// upper_body.push_back("TibiaL");
	// upper_body.push_back("TalusL");
	
	TiXmlDocument doc;
	TiXmlElement* muscle_elem = new TiXmlElement("Muscle");
	doc.LinkEndChild(muscle_elem);

	for(int i =0;i<mcs.size();i++)
	{
		TiXmlElement* unit_elem = new TiXmlElement("Unit");
		// std::cout<<mcs[i].mName<<" ";
		auto& uc = ucs.at(mcs[i].mName);

		unit_elem->SetAttribute("name",mcs[i].mName);
		unit_elem->SetAttribute("f0",std::to_string(uc.f0));
		unit_elem->SetAttribute("lm",std::to_string(uc.lm));
		unit_elem->SetAttribute("lt",std::to_string(uc.lt));
		unit_elem->SetAttribute("pen_angle",std::to_string(uc.pen_angle));
		unit_elem->SetAttribute("lmax",std::to_string(uc.l_max));
		bool is_lower_body = true;
		for(int j =0;j<mcs[i].mAnchors.size();j++)
		{
			TiXmlElement* waypoint_elem = new TiXmlElement("Waypoint");

			for(int k =0;k<upper_body.size();k++)
				if(mcs[i].mAnchors[j].first == upper_body[k])
					is_lower_body = false;

			// if(mcs[i].mAnchors[j].first == "Pelvis")
			// 	is_lower_body = true;
			// else
			// 	is_lower_body = false;
				
			if(uc.l_max<0.0)
				is_lower_body = false;
			waypoint_elem->SetAttribute("body",mcs[i].mAnchors[j].first);
			waypoint_elem->SetAttribute("p",toString(mcs[i].mAnchors[j].second));
			unit_elem->LinkEndChild(waypoint_elem);	
		}
		if(is_lower_body){
			std::cout<<mcs[i].mName<<" "<<uc.f0<<std::endl;
			muscle_elem->LinkEndChild(unit_elem);
		}
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile("../model/muscle.xml");
	
	return 0;
}