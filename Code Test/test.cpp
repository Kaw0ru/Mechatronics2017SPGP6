// constructors and derived classes
#include <iostream>
using namespace std;

class Dog{
public:
	Dog(){
		cout << "Dog has been initialized!" << endl;
	}
	~Dog(){
		cout << "Dog has been destroied!" << endl;
	}
	void SetBreed(int bre){
		breed = bre;
	}
	int GetBreed(){
		return breed;
	}
private:
	int breed;
};

class BadDog: public Dog {
public:
	BadDog (){
		cout << "Bad dog has been initialized!" << endl;
		this->SetBreed(0);
	}
	~BadDog(){
		cout << "Bad dog has been destroied!" << endl;
	}
	void SetBite(int numOfBite){
		bite = numOfBite;
	}
	int GetBite(){
		return bite;
	}
private:
	int bite;
};

int main(){
	Dog vacca;
	vacca.SetBreed(1);
	cout << "Vacca's breed " << vacca.GetBreed() << endl;

	BadDog ICC;
	// ICC.SetBreed(12);
	ICC.SetBite(3);
	cout << "ICC's breed " << ICC.GetBreed() << endl;
	cout << "ICC's number of bite " << ICC.GetBite() << endl;
	return 0;
}