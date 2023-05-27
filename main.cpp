#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <bits/stdc++.h>
#include <vector>

static bool isFloatNumber(const std::string& string){
    std::string::const_iterator it = string.begin();
    bool decimalPoint = false;
    int minSize = 0;
    if(string.size()>0 && (string[0] == '-' || string[0] == '+')){
      it++;
      minSize++;
    }
    while(it != string.end()){
      if(*it == '.'){
        if(!decimalPoint) decimalPoint = true;
        else break;
      }else if(!std::isdigit(*it) && ((*it!='f') || it+1 != string.end() || !decimalPoint)){
        break;
      }
      ++it;
    }
    return string.size()>minSize && it == string.end();
  }
int main(){
	std::vector<std::vector<long double>> message(8, std::vector<long double>(12));
	int i=0, j=0;
	for(int i=0; i<5; i++){
		for(int j=0; j<12; j++){
			message[i][j]=0.0;
		}
	}


	std::ifstream myfile;
	myfile.open("C:\\Users\\dimak\\Downloads\\test2.waypoints");

	std::string myline;
	if ( myfile.is_open() ) {
		while (std::getline (myfile, myline)) {
			j=0;
			
			std::cout.flush();
			std::stringstream ss(myline);  
    		std::string word;
			while (ss >> word) { // Extract word from the stream.
        		if(isFloatNumber(word)){
					message[i][j]=std::stod(word);
				}
				std::cout.flush();
				// std::cout<<i<<" "<<message[i][j]<<" "<<myfile.good()<<std::endl;
				j++;
				
    		}
			i++;
		}
	}
	myfile.close();
	for(int k1=0; k1<8; k1++){
		for(int k2=0; k2<12; k2++){
			std::cout<<std::fixed << std::setprecision(6)<<message[k1][k2]<<' ';
		}
		std::cout<<'\n';
	}

	return 0;
}