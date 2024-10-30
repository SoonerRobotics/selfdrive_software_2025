"use client";

import { socket } from "@/lib/socket"; 
import { useEffect, useRef } from "react";

export default function Home() {
    const motorFeedbackRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        function onConnect() {
            console.log('connected');
        }

        function onDisconnect() {
            console.log('disconnected');
        }

        function onMessage(data: any) {
            if (motorFeedbackRef.current) {
                motorFeedbackRef.current.innerHTML = JSON.stringify(data, null, 2);
            }
        }

        socket.on('connect', onConnect);
        socket.on('disconnect', onDisconnect);
        socket.on('motor_feedback', onMessage);

        return () => {
            socket.off('connect', onConnect);
            socket.off('disconnect', onDisconnect);
            socket.off('motor_feedback', onMessage);
        }
    }, []);

    return (
        <div>
            {
                socket.connected ? "Connected" : "Disconnected"
            }

            <div ref={motorFeedbackRef}></div>

            <div className="flex flex-row">
                <img src="http://localhost:4029/left_cam" />
                <img src="http://localhost:4029/right_cam" />
            </div>
        </div>
    );
}
