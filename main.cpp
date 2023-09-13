#include <queue>
#include <algorithm>
#include <climits>
#include <iostream>
#include <future>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <mutex>

std::mutex logMutex;

void logMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(logMutex);
    std::cout << message;
}

enum class MovementType {
    UP,
    DOWN,
    IDLE
};

class FloorController;

class ExternalCommand {
public:
    MovementType type;
    unsigned int floor;
    ExternalCommand(MovementType _type, unsigned int _floor) : type(_type), floor(_floor) {}
};

class Elevator {
private:
    unsigned int myNumber;
    unsigned int myFloor = 0;
    MovementType myState = MovementType::IDLE;
    FloorController* parentFloor;
    std::deque<unsigned int> floorsToStop;
public:
    explicit Elevator(unsigned int _myNumber, FloorController* _parentFloor) : myNumber(_myNumber), parentFloor(_parentFloor) {}
    void buttonPressed(unsigned int floor);
    void receiveCommand(ExternalCommand cmd) {
        myState = cmd.type;
        if(floorsToStop.empty()) {
            floorsToStop.push_back(cmd.floor);
        } else {
            auto insertPos = std::lower_bound(floorsToStop.begin(), floorsToStop.end(), cmd.floor,
                                              std::greater<unsigned int>());
            floorsToStop.insert(insertPos, cmd.floor);
        }
    }

    std::pair<MovementType, unsigned int> getState() const {
        return std::make_pair(myState, myFloor);
    }

    // Movement simulation functions
    void moveElevator();
};

class FloorController : std::enable_shared_from_this<FloorController> {
private:
    std::vector<std::pair<MovementType, unsigned int>> elevatorStates;
    std::queue<ExternalCommand> cmd;
public:
    std::vector<Elevator> elevators;

    explicit FloorController(unsigned int elevatorsAmount) {
        for(int i = 0; i < elevatorsAmount; i++) {
            elevators.emplace_back(i, this);
            elevatorStates.emplace_back(std::make_pair(MovementType::IDLE, 0));
        }
    }

    void processUpdateFromElevator(unsigned int elevatorId, MovementType type, unsigned int floor) {
        elevatorStates[elevatorId] = std::make_pair(type, floor);
        if(!cmd.empty()) {
            auto currentCmd = cmd.front();

            auto elevatorFitId = findBestFitElevator(currentCmd.floor);
            if (elevatorFitId != -1) {
                elevators.at(elevatorFitId).receiveCommand({type, floor});
                cmd.pop();
            }
        }
    }

    int findBestFitElevator(unsigned int floor) {
        unsigned int closestValue = UINT_MAX;
        int closestId = -1;

        for (size_t i = 0; i < elevatorStates.size(); ++i) {
            if (elevatorStates.at(i).first == MovementType::IDLE && std::abs(static_cast<int>(elevatorStates[i].second) - static_cast<int>((floor))) <= std::abs(static_cast<int>(closestValue) - static_cast<int>(floor))) {
                closestValue = elevatorStates.at(i).second;
                closestId = i;
            }
        }
        return closestId;
    }

    void buttonPressed(unsigned int floor, MovementType type) {
        bool cmdDone = false;

        switch (type) {
            case MovementType::DOWN:
                for(auto& elevator : elevators) {
                    auto curElevatorState = elevator.getState();

                    if(curElevatorState.first == MovementType::DOWN && curElevatorState.second > floor) {
                        elevator.receiveCommand({type, floor});
                        cmdDone = true;
                        break;
                    }
                }
                if(!cmdDone) {
                    auto elevatorFitId = findBestFitElevator(floor);

                    if (elevatorFitId >= 0) {
                        elevators.at(elevatorFitId).receiveCommand({type, floor});
                    } else {
                        cmd.emplace(type, floor);
                    }
                }
                break;
            case MovementType::UP:
                if(cmd.empty()) {
                    auto elevatorFitId = findBestFitElevator(floor);

                    if (elevatorFitId >= 0) {
                        elevators.at(elevatorFitId).receiveCommand({type, floor});
                    } else {
                        cmd.emplace(type, floor);
                    }
                } else {
                    cmd.emplace(type, floor);
                }
                break;
        }
    }
};

void Elevator::moveElevator() {
    // Wait timeout as Elevator is moving to N floor
    logMessage("Elevator[" + std::to_string(myNumber) + "] on floor [" + std::to_string(myFloor) + "]\n");

    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if(!floorsToStop.empty()) {
            auto aimFloor = floorsToStop.front();

            logMessage("Elevator[" + std::to_string(myNumber) + "] request to floor [" + std::to_string(aimFloor) + "]\n");
            if(myFloor < aimFloor) {
                myState = MovementType::UP;
            } else if(myFloor > aimFloor) {
                myState = MovementType::DOWN;
            }
            parentFloor->processUpdateFromElevator(myNumber, myState, myFloor);

            while (aimFloor != myFloor) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                switch (myState) {
                    case MovementType::UP:
                        myFloor++;
                        break;
                    case MovementType::DOWN:
                        myFloor--;
                        break;
                    case MovementType::IDLE:
                        break;
                }
                logMessage("Elevator[" + std::to_string(myNumber) + "] on floor [" + std::to_string(myFloor) + "]\n");
            }
            floorsToStop.pop_front();
            myState = MovementType::IDLE;
            parentFloor->processUpdateFromElevator(myNumber, myState, myFloor);
            std::srand(static_cast<unsigned int>(std::time(nullptr)));
            int random_number = std::rand() % (11);
            int random_press = std::rand() % (2);
            if(random_press) {
                buttonPressed(random_number);
                logMessage("Elevator[" + std::to_string(myNumber) + "] button pressed to floor [" + std::to_string(random_number) + "]\n");
            }
        }
    }
}

void Elevator::buttonPressed(unsigned int floor) {
    if(myFloor < floor) {
        myState = MovementType::UP;
    } else if(myFloor > floor) {
        myState = MovementType::DOWN;
    }
    parentFloor->processUpdateFromElevator(myNumber, myState, myFloor);
    floorsToStop.push_back(floor);
}

int main() {
    auto controller = new FloorController(2);

    std::future<void> firstElevator = std::async(std::launch::async, [controller]() {
        controller->elevators.at(0).moveElevator();
    });
    std::future<void> secondElevator = std::async(std::launch::async, [controller]() {
        controller->elevators.at(1).moveElevator();
    });

    controller->buttonPressed(5, MovementType::DOWN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    controller->buttonPressed(6, MovementType::DOWN);

    // Wait for the elevator tasks to complete
    firstElevator.get();
    secondElevator.get();
    delete controller;
    return 0;
}
