export interface Station {
  id: number;
  name?: string;
  cameraIds?: number[];
  objDetections: string[];
  active: boolean;
  imageUrl: string;
  createdAt: string;
  updatedAt: string;
}
