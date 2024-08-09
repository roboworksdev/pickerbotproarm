#ifndef _CMDPANEL_H_
#define _CMDPANEL_H_

#include <vector>
#include <deque>
#include "common/utilities/loop.h"
#include "message/arm_common.h"


enum class KeyPress{
    RELEASE,
    PRESS,
    REPEAT
};


enum class ActionType{
    EMPTY,
    STATE,
    VALUE
};


struct KeyCmd{
    std::string c;
    KeyPress keyPress;
};


class KeyAction{
public:
    KeyAction(ActionType type);
    virtual ~KeyAction(){};
    ActionType getType(){return _type;}
protected:
    ActionType _type;
};


class StateAction : public KeyAction{
public:
    StateAction(std::string c, int state, KeyPress press = KeyPress::PRESS);
    virtual ~StateAction(){};
    int getState(){return _state;};
    bool handleCmd(KeyCmd keyCmd, int &state);
protected:
    int _state;
    KeyCmd _keyCmdSet;
};

class EmptyAction : public StateAction{
public:
    EmptyAction(int state);
    ~EmptyAction(){};
private:

};

class ValueAction : public KeyAction{
public:    
    ValueAction(std::string cUp, std::string cDown, double deltaValue, double initValue = 0.0);
    ValueAction(std::string cUp, std::string cDown, std::string cGoZero, double deltaValue, double initValue = 0.0);
    ValueAction(std::string cUp, std::string cDown, double deltaValue, double limit1, double limit2, double initValue = 0.0);
    ValueAction(std::string cUp, std::string cDown, std::string cGoZero, double deltaValue, double limit1, double limit2, double initValue = 0.0);

    ~ValueAction(){};
    bool handleCmd(KeyCmd keyCmd);
    void setDt(double dt);
    double getValue();
    double getDValue();
    double getDirection(){return _changeDirection;};
    void setValue(double value){_value = value;}
private:
    double _value;
    double _changeDirection=0.0;
    double _dV = 0.0; //delta value per delta time
    double _dt = 0.0;
    double _dVdt = 0.0; // delta value per second
    double _dVdtf = 0.0; // delta value per second after fliter
    double _lim1, _lim2;
    bool _hasLim = false;
    bool _hasGoZero = false;

    KeyCmd _upCmdSet;
    KeyCmd _downCmdSet;
    KeyCmd _goZeroCmdSet;

};


class CmdPanel{
public:
    CmdPanel(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1, double dt = 0.002);
    virtual ~CmdPanel();
    virtual int getState(size_t channelID = 0);
    std::vector<double> getValues() {return _values;};
    std::vector<double> getDValues() {return _dValues;};
    std::vector<double> getDirections() {return _changeDirections;};
    void setValue(std::vector<double> values);
    void setValue(double value, size_t id);
    virtual std::string getString(std::string slogan);
    virtual std::vector<double> stringToArray(std::string slogan);
    virtual std::vector<std::vector<double> > stringToMatrix(std::string slogan);
    virtual SendCmd getSendCmd();
    virtual void setRecvState(RecvState& recvState){};
    void start(){_runThread->start();}
protected:
    virtual void _read() = 0;
    void _run();
    void _updateState();
    void _pressKeyboard();
    void _releaseKeyboard();

    LoopFunc *_runThread;
    LoopFunc *_readThread;

    std::vector<StateAction> _stateEvents;
    std::vector<ValueAction> _valueEvents;

    EmptyAction _emptyAction;
    size_t _actionNum = 0;
    size_t _stateNum = 0;
    size_t _valueNum = 0;
    size_t _channelNum;
    std::vector<double> _values;
    std::vector<double> _dValues;
    std::vector<double> _changeDirections;
    int _state;
    std::vector<std::deque<int>> _stateQueue;
    std::vector<int> _outputState;
    std::vector<bool> _getState;
    double _dt;
    KeyCmd _keyCmd;
    std::string _cPast = "";

    bool _running = true;
};

#endif