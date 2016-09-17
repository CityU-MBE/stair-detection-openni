#ifndef GENERICSTATE_H
#define GENERICSTATE_H

template <typename StateMachine, class State, class SuperState = StateMachine>
class GenericState
{
public:
    static void init(State *&state, State &initState) {
        state = &initState;
        initState.entry();
    }

protected:
    explicit GenericState(StateMachine &m, State *&state) :
        m(m), s(m), state(state) {}

    explicit GenericState(StateMachine &m, State *&state, SuperState &s) :
        m(m), s(s), state(state) {}

    void change(State &targetState) {
        exit();
        init(state, targetState);
    }

private:
    virtual void entry() {}
    virtual void exit() {}

protected:
    StateMachine &m;
    SuperState &s;

private:
    State *&state;
};

#endif // GENERICSTATE_H
