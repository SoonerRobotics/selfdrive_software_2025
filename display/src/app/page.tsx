"use client";

import IdleScreen from "@/components/IdleScreen";
import { socket } from "@/lib/socket"; 
import { useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";

export default function Home() {
    const motorFeedbackRef = useRef<HTMLDivElement>(null);
    const [render, setRender] = useState(false);

    useEffect(() => {
        function onConnect() {
            console.log('connected');

            setRender(!render);
        }

        function onDisconnect() {
            console.log('disconnected');

            setRender(!render);
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

    if (!socket.connected)
    {
        return <IdleScreen />
    }

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
