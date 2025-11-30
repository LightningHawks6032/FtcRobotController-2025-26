package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

public class StateMachine <T> {
    private class Node {
        T type;
        public Runnable onEnter, onExit, onLoop;

        public Node(T _type, Runnable _onEnter, Runnable _onExit, Runnable _onLoop) {
            type = _type;
            onEnter = _onEnter;
            onExit = _onExit;
            onLoop = _onLoop;
        }
    }

    private class Arrow {
        public T target;
        public Supplier<Boolean> condition;

        public Arrow(T _target, Supplier<Boolean> _condition) {
            target = _target;
            condition = _condition;
        }
    }

    HashMap<T, Node> nodes;
    HashMap<T, ArrayList<Arrow>> graph;
    Node currentNode;

    public void loop() {
        currentNode.onLoop.run();
        if (!graph.containsKey(currentNode.type)) {return;}

        for (Arrow arrow : graph.get(currentNode.type)) {
            if (arrow.condition.get()) {
                currentNode.onExit.run();
                currentNode = nodes.get(arrow.target);
                assert currentNode != null;
                currentNode.onEnter.run();
                return;
            }
        }
    }

    public void addNode(T type, Runnable _onEnter, Runnable _onExit, Runnable _onLoop) {
        nodes.put(type, new Node(type, _onEnter, _onExit, _onLoop));
    }

    static Runnable _emptyRunnable = () -> {};
    public void addNode(T type) {

        addNode(type, _emptyRunnable, _emptyRunnable, _emptyRunnable);
    }

    public void addArrow(T source, T target, Supplier<Boolean> condition) {
        Arrow newArrow = new Arrow(target, condition);

        if (graph.containsKey(source)) {
            graph.get(source).add(newArrow);
        }
        else {
            graph.put(source, new ArrayList<>());
            graph.get(source).add(newArrow);
        }
    }

    public void setCurrentNode(T node) {
        currentNode = nodes.get(node);
        currentNode.onEnter.run();
    }

    public T getCurrentNode() {
        return currentNode.type;
    }
}
