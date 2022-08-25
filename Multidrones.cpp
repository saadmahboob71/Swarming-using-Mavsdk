//./Tools/gazebo_sitl_multiple_run.sh
//cmake -Bbuild -H.
//cmake --build build -j4
//connect multiple vehicles and make them take off and land in parallel.
//build/multiple_drones udp://:14540 udp://:14541 udp://:14542
//

#include <random>
#include <cstring>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <cstdint>
#include <atomic>
#include <iostream>
#include <thread>
#include <future>
#include <chrono>


using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

static void takeoff(std::shared_ptr<System> system);
static void pso(std::shared_ptr<System> system1,std::shared_ptr<System> system2,std::shared_ptr<System> system3);
static void land(std::shared_ptr<System> system);
double pos1[3],pos2[3],pos3[3];
int num_dimensions=3;
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(-1,1); //doubles from -1 to 1
std::uniform_real_distribution<double> distri(0,1); //doubles from 0 to 1
double costFunc(double x[3])//3 because of longitude, latitude,height
{
    double total=0;
    for (int i=0;i<3;i++)
    {
        total+=x[i]*x[i];
    }
    return total;
}
class Particle
{
    double pos[3],velocity[3],pos_best[3],err_best,err;//pos o,1,2 are latitude,longitude and height respectively

    public:
    Particle(){err_best=-1;err=-1;velocity[0]= distribution(generator);velocity[1]= distribution(generator);velocity[2]= distribution(generator);};
    Particle(double longit,double latit,double altit){pos[0]=longit;pos[1]=latit;pos[2]=altit;err_best=-1;err=-1;velocity[0]= distribution(generator);velocity[1]= distribution(generator);velocity[2]= distribution(generator);};
    void setpos(double uppos[3])
    {
        pos[0]=uppos[0];pos[1]=uppos[1];pos[2]=uppos[2];
    };
   
    void evaluate()
    {
        //evaluate current fitness
        err=costFunc(pos);
        if (err < err_best || err_best==-1)
        {
            memcpy(pos_best,pos,sizeof(pos));
            err_best=err;
        }
    };
    double* getpos()
    {
        return pos;
    };
    void printpos()
    {
    std::cout<<"In class current-position is: Longitude "<< pos[0]<<" Lattitude "<<pos[1]<<" Height "<<pos[2]<<"\n";
    };
    void update_velocity(double* pos_best_g)
    {
        double w=0.5;       // constant inertia weight (how much to weigh the previous velocity)
        double c1=1;        //cognative constant
        double c2=2;        // social constant

        for (int i=0;i<num_dimensions;i++)
        {
            double r1=distri(generator);
            double r2=distri(generator);

            double vel_cognitive=c1*r1 * (pos_best[i] - pos[i]);
            double vel_social=c2*r2 * (pos_best_g[i] - pos[i]);
            //std::cout<<"VELOCITY NO ATM IS "<<i<<"\n" ;
            velocity[i]=w*velocity[i]+vel_cognitive+vel_social;
        }
    };
    void update_position(double bounds[3][3])
    {
        for (int i=0;i<num_dimensions;i++)
        {
            pos[i]=pos[i]+velocity[i];
       
            // adjust maximum position if necessary
            /*
            if (pos[i]>bounds[i][1])
            {
                pos[i]=bounds[i][1];
            }
            // adjust minimum position if neseccary
            if (pos[i] < bounds[i][0])
            {
                pos[i]=bounds[i][0];
            }*/
        }
    };
    double geterr()
    {
        return err;
    };
};

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url_1> [<connection_url_2> ...]\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}
//double initial[2]={3,2}               // initial starting location[x1,x2...]

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Please specify connection\n";
        usage(argv[0]);
        return 1;
    }
   
    Mavsdk mavsdk;

    size_t total_udp_ports = argc - 1;
std::cout<<"\n-------> ARGC:"<<argc;
std::cout<<"\n-------> ARG 0 :"<<argv[0];
    // the loop below adds the number of ports the sdk monitors.
    for (int i = 1; i < argc; ++i) {
        ConnectionResult connection_result = mavsdk.add_any_connection(argv[i]);
        if (connection_result != ConnectionResult::Success) {
            std::cerr << "Connection error: " << connection_result << '\n';
            return 1;
        }
    }

    std::atomic<size_t> num_systems_discovered{0};

    std::cout << "Waiting to discover system...\n";
    mavsdk.subscribe_on_new_system([&mavsdk, &num_systems_discovered]() {
        const auto systems = mavsdk.systems();

        if (systems.size() > num_systems_discovered) {
            std::cout << "Discovered system\n";
            num_systems_discovered = systems.size();
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (num_systems_discovered != total_udp_ports) {
        std::cerr << "Not all systems found, exiting.\n";
        return 1;
    }

    std::vector<std::thread> threads;
    int i=0;
    auto l = mavsdk.systems().begin();
    const auto systems = mavsdk.systems();
    const auto system1=*(l++),system2=*(l++),system3=*l;
   

    for (auto system : mavsdk.systems()) {
   
        std::thread t(&takeoff, std::ref(system));
        threads.push_back(std::move(t));
        sleep_for(seconds(1));
        std::cout<<"System at the moment is"<<system<<" no is "<<i<<"\n";
        if(i==0)
        {
           auto system1=std::ref(system);
        }
        else if (i==1)
        {
           auto system2=std::ref(system);
        }
        else
        {
           auto system3=std::ref(system);
        }
       
        i++;
    }
    i=0;

    /*for (auto& t : threads) {
        t.join();
        std::cout<<"at the moment is"<<system<<"no is "<<i;
    }*/
   
    pso(system1,system2,system3);
    return 0;
}
static void pso(std::shared_ptr<System> system1,std::shared_ptr<System> system2,std::shared_ptr<System> system3)
{
    double bounds[3][3]={(46,52),(4,12),(-10,20)};  // input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
    int num_particles=3;// no of drones
    int maxiter=15;
    double err_best_g=-1;//best error
    double *pos_best_g,*curr_pos;pos1[0]=0;pos1[1]=0;pos1[2]=0;
   
    std::cout<<"System at the moment is"<<system1<<" no is "<<0<<"\n";
    std::cout<<"System at the moment is"<<system2<<" no is "<<1<<"\n";    
    std::cout<<"System at the moment is"<<system3<<" no is "<<2<<"\n";

    auto telemetry1 = std::make_shared<Telemetry>(system1);
    auto action1 = Action{system1};
    auto offboard1 = Offboard{system1};
    auto telemetry2 = std::make_shared<Telemetry>(system2);
    auto action2 = Action{system2};
    auto offboard2 = Offboard{system2};
    auto telemetry3 = std::make_shared<Telemetry>(system3);
    auto action3 = Action{system3};
    auto offboard3 = Offboard{system3};
    const Telemetry::Result set_rate_result= telemetry1->set_rate_position(1.0);
    const Telemetry::Result set_rate_result1= telemetry2->set_rate_position(1.0);
    const Telemetry::Result set_rate_result2= telemetry3->set_rate_position(1.0);


    int i=0;
    //double pos1[3],pos2[3],pos3[3];
    pos1[0]=0;pos1[1]=0;pos1[2]=0;
    telemetry1->subscribe_position([](Telemetry::Position position){std::cout << "Altitude 1: " << position.relative_altitude_m << " m\n";
    std::cout << "Latitude 1: " << position.latitude_deg<< " deg\n";
    std::cout << "Longitude 1: " << position.longitude_deg << " deg\n";
    pos1[0]=position.latitude_deg;pos1[1]=position.longitude_deg;pos1[2]=position.relative_altitude_m;  
 
});
 telemetry2->subscribe_position([](Telemetry::Position position){std::cout << "Altitude 2: " << position.relative_altitude_m << " m\n";
 std::cout << "Latitude 2: " << position.latitude_deg<< " deg\n";
 std::cout << "Longitude 2: " << position.longitude_deg << " deg\n";
  pos2[0]=position.latitude_deg;pos2[1]=position.longitude_deg;pos2[2]=position.relative_altitude_m;  
});
 telemetry3->subscribe_position([](Telemetry::Position position){std::cout << "Altitude 3: " << position.relative_altitude_m << " m\n";
 std::cout << "Latitude 3: " << position.latitude_deg<< " deg\n";
 std::cout << "Longitude 3: " << position.longitude_deg << " deg\n";
  pos3[0]=position.latitude_deg;pos3[1]=position.longitude_deg;pos3[2]=position.relative_altitude_m;  
});
    const Telemetry::Position posi1=telemetry1->position();
    const Telemetry::Position posi2=telemetry2->position();
    const Telemetry::Position posi3=telemetry3->position();
    pos3[0]=posi3.latitude_deg;pos3[1]=posi3.longitude_deg;pos3[2]=posi3.relative_altitude_m;
    pos2[0]=posi2.latitude_deg;pos2[1]=posi2.longitude_deg;pos2[2]=posi2.relative_altitude_m;
    pos1[0]=posi1.latitude_deg;pos1[1]=posi1.longitude_deg;pos1[2]=posi1.relative_altitude_m;

    Particle drone1(pos1[0],pos1[1],pos1[2]);
    Particle drone2(pos2[0],pos2[1],pos2[2]);
    Particle drone3(pos3[0],pos3[1],pos3[2]);
    
    std::cout<<"LATITUDEEE "<<pos1[0]<<"LOOONGIIITUDEEE "<<pos1[1]<<"HEIGHT "<<pos1[2];
    //drone1.printpos();
   
    while (i < maxiter)
    {
            //print i,err_best_g
            //cycle through particles in swarm and evaluate fitness

        for(int j=0;j<3;j++)
        {
       
       
        if(j==0)
        {
            drone1.setpos(pos1);
            drone1.evaluate();
            std::cout<<"Evaluated \n";
            curr_pos=drone1.getpos();
            std::cout<<"current-position is: Longitude "<< curr_pos[0]<<"Lattitude "<<curr_pos[1]<<"Height "<<curr_pos[2]<<"\n";
                    // determine if current particle is the best (globally)
            if ((drone1.geterr() < err_best_g) || (err_best_g == -1))
                {
                    pos_best_g = drone1.getpos();
                    err_best_g = drone1.geterr();
                    std::cout<<"best-position : Longitude "<< pos_best_g[0]<<"Lattitude "<<pos_best_g[1]<<"Height "<<pos_best_g[2]<<"\n";
                    std::cout<<"Best-Err"<< err_best_g<<"\n";
            }
               
        }
        if(j==1)
        {
            drone2.setpos(pos2);
            drone2.evaluate();
            std::cout<<"Evaluated \n";
            curr_pos=drone2.getpos();
            std::cout<<"current-position is: Longitude "<< curr_pos[0]<<"Lattitude "<<curr_pos[1]<<"Height "<<curr_pos[2]<<"\n";
                    // determine if current particle is the best (globally)
            if ((drone2.geterr() < err_best_g) || (err_best_g == -1))
            {
                    //memcpy(pos_best_g,swarm[j].getpos,sizeof(swarm[j].getpos));
                pos_best_g = drone2.getpos();
                err_best_g = drone2.geterr();
                std::cout<<"best-position : Longitude "<< pos_best_g[0]<<"Lattitude "<<pos_best_g[1]<<"Height "<<pos_best_g[2]<<"\n";
                std::cout<<"Best-Err"<< err_best_g<<"\n";
            }
               
        }
        if(j==2)
        {
            drone3.setpos(pos3);
            drone3.evaluate();
            std::cout<<"Evaluated \n";
            curr_pos=drone3.getpos();
            std::cout<<"current-position is: Longitude "<< curr_pos[0]<<"Lattitude "<<curr_pos[1]<<"Height "<<curr_pos[2]<<"\n";
                    // determine if current particle is the best (globally)
            if ((drone3.geterr() < err_best_g) || (err_best_g == -1))
            {
                    //memcpy(pos_best_g,swarm[j].getpos,sizeof(swarm[j].getpos));
                pos_best_g = drone3.getpos();
                err_best_g = drone3.geterr();
                std::cout<<"Here\n";
                std::cout<<"best-position : Longitude "<< pos_best_g[0]<<"Lattitude "<<pos_best_g[1]<<"Height "<<pos_best_g[2]<<"\n";
                std::cout<<"Best-Err"<< err_best_g<<"\n";
            }
               
                }
        }
                // cycle through swarm and update velocities and position
        double longitude[3],latitude[3],height[3];
        //temporary initilaizing
        //pos_best_g[0]=0;pos_best_g[1]=0;pos_best_g[2]=0;
        //std::cout<<"Completed 1st loop\n";
        for(int l=0;l<num_particles;l++)
        {
            if(l==0)
            {
           
            drone1.update_velocity(pos_best_g);
            std::cout<<"Velocity updated \n";
            drone1.update_position(bounds);
            std::cout<<"Position updated \n";
            drone1.printpos();
            curr_pos=drone1.getpos();
            //action1.goto_location(curr_pos[0],curr_pos[1],curr_pos[2],0); //const;
            //const Action::Result result = action1.goto_location(48,8,5,0);
             //if (result != Action::Result::Success) {
        //std::cerr << "Action  failed: " << result << '\n';}
            //Action::ResultCallback Result = action1.goto_location(48,8,5,0);
           
            std::cout<<"MOVING\n";
            sleep_for(seconds(10));
            std::cout<<"UPDATE POS for system no ------ Longitude "<<l<< curr_pos[0]<<" Lattitude "<<curr_pos[1]<<" Height "<<curr_pos[2]<<"\n";
            }
            if(l==1)
            {
            drone2.update_velocity(pos_best_g);
            std::cout<<"Velocity updated \n";
            drone2.update_position(bounds);
            std::cout<<"Position updated \n";
            curr_pos=drone2.getpos();
            drone2.printpos();
            //using mavsdk::Action::ResultCallback =
            //const Action::Result result = action2.goto_location(48.004,8.002,5,0);
             //if (result != Action::Result::Success) {
        //std::cerr << "Action  failed: " << result << '\n';
        //}
            //action2.goto_location(48,8,5,0);
            std::cout<<"MOVING\n";
            //action2.goto_location(curr_pos[0],curr_pos[1],curr_pos[2],0); //const;
            sleep_for(seconds(10));
            std::cout<<"UPDATE POS for system no ------ Longitude "<<l<< curr_pos[0]<<"Lattitude "<<curr_pos[1]<<"Height "<<curr_pos[2]<<"\n";
            }
            if(l==2)
            {
            drone3.update_velocity(pos_best_g);
            std::cout<<"Velocity updated \n";
            drone3.update_position(bounds);
            std::cout<<"Position updated \n";
            drone3.printpos();
            curr_pos=drone3.getpos();
            //sing mavsdk::Action::ResultCallback =action3.goto_location(46,7.002,4,0);
            //const Action::Result result = action3.goto_location(48.0008,8.0004,5,0);
             //if (result != Action::Result::Success) {
        //std::cerr << "Action  failed: " << result << '\n';
        //}
            //std::cout<<"MOVING\n";
            //action3.goto_location(curr_pos[0],curr_pos[1],curr_pos[2],0); //const;
            sleep_for(seconds(10));
            std::cout<<"UPDATE POS for system no ------ Longitude "<<l<< curr_pos[0]<<"Lattitude "<<curr_pos[1]<<"Height "<<curr_pos[2]<<"\n";
            }
           

        }
    }
    std::cout<<"FINAL:\n";
    std::cout<<"best-position : Longitude "<< pos_best_g[0]<<"Lattitude "<<pos_best_g[1]<<"Height "<<pos_best_g[2]<<"\n";
    std::cout<<err_best_g;
    sleep_for(seconds(20));
    return;

}

void takeoff(std::shared_ptr<System> system)
{
    auto telemetry = Telemetry{system};
    auto action = Action{system};
    auto offboard = Offboard{system};

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);

    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed:" << set_rate_result << '\n';
        return;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
        std::cout << "Latitude: " << position.latitude_deg << " deg\n";
        std::cout << "Longitude: " << position.longitude_deg << " deg\n";
});

    // Check if vehicle is ready to arm
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed:" << arm_result << '\n';
    }

    // Take off
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed:" << takeoff_result << '\n';
    }
    //sleep_for(seconds(3));
    //const Action::Result result = action.goto_location(47.3978,8.54565,10,0);
      //       if (result != Action::Result::Success) {
        //std::cerr << "Action  failed: " << result << '\n';
        //}
    /*
    const mavsdk::Offboard::VelocityNedYaw stay{};
    offboard.set_velocity_ned(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
    }
    //Moving vehicle 5m north and 5m up

    std::cout << "Offboard started\n";
    mavsdk::Offboard::PositionNedYaw position_ned_yaw{};
    position_ned_yaw.north_m=5.0;
    position_ned_yaw.east_m=0.0;
    position_ned_yaw.down_m=-5.0;
    position_ned_yaw.yaw_deg=0;
    offboard.set_position_ned(position_ned_yaw);
    sleep_for(seconds(5));


    // Let it hover for a bit before landing again.
    sleep_for(seconds(20));
    offboard_result = offboard.stop();

    std::cout << "Landing...\n";
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed:" << land_result << '\n';
    }
   

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.

    sleep_for(seconds(5));
    std::cout << "Finished...\n";*/
    return;
}
void land(std::shared_ptr<System> system)
{
    auto telemetry = Telemetry{system};
    auto action = Action{system};
    auto offboard = Offboard{system};

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
std::cout << "Landing...\n";
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed:" << land_result << '\n';
    }
   

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.

    sleep_for(seconds(5));
    std::cout << "Finished...\n";
    return;
}
    