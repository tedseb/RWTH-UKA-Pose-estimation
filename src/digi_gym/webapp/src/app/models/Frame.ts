import { Camera } from "./Camera";
import { Station } from "./Station";

export interface Frame {
    id: number,
    frame_box: [[number, number], [number, number]],
    createdAt: string, 
    updatedAt: string,
    camera_station_mapping: {
        id: number,
        createdAt: string, 
        updatedAt: string,
        stationId: number,
        station: Station,
        cameraId: number,
        camera: Camera,
    }
}