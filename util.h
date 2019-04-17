#pragma once

enum LaunchState {
    PRE_LAUNCH = 0b011,
    ON_GROUND = 0b101,
    LAUNCHED = 0b110,
    PASSED_MIN_ALT = 0b001,
    PASSED_APOGEE = 0b010,
    PASSED_MAIN_CHUTE = 0b100
};

struct Node {
    Node* previous;
    Node* next;
    double value;

    Node(Node* p, double v) : previous(p), value(v) {}

    static Node* createList(int length) {
        Node* first = new Node(nullptr, 0);
        Node* last = first; 
        for (int i = 0; i < length; i++) {
            last = new Node(last, 0);
            last->previous->next = last;
        }
        first->previous = last;
        last->next = first;
        return first;
    }
};
