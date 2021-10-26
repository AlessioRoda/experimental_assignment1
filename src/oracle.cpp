#include "ros/ros.h"
#include "experimental_assignment1/Hint.h"

std::string people[6]={'Col.Mustard', 'Miss.Scarlett', 'Mrs.Peacock', 'Mrs.White', 'Prof.Plum', 'Rev.Green'};
std::string places[9]={'Ballroom', 'Biliard_Room', 'Conservatory', 'Dining_Room', 'Hall', 'Kitchen', 'Library', 'Lounge','Study'};
std::string weapons[6]={'Candlestick', 'Dagger','LeadPipe', 'Revolver', 'Rope', 'Spanner'};
std::string solution[3];


/** Gets the params for the interval from the request, then call the function randMToN to obtain a number in that interval**/
string hint (experimental_assignment1::Hint::Request &req, experimental_assignment1::Hint::Response &res){



    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/** Initialize the node and create a new custom service of type RandomPosition on /position_server **/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "oracle");
   ros::NodeHandle n;

/** Defines a solution starting from the 3 arrays and remove these elements from them**/
   int index = rand() / ( RAND_MAX /6 );
   solution[0]=people[index];
   std::remove(std::begin(people), std::end(people), solution[0]);
   index= rand() / ( RAND_MAX /9 );
   solution[1]=places[index];
   std::remove(std::begin(places), std::end(places), solution[1]);
   index= rand() / ( RAND_MAX /6 ); 
   solution[2]=weapons[index];
   std::remove(std::begin(weapons), std::end(weapons), solution[2]);

   ros::ServiceServer service= n.advertiseService("/state_machine", hint);
   
   ros::spin();

   return 0;
}