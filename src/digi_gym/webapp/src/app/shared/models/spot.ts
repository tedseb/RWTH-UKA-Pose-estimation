export class Spot {

    id: number;
    active: boolean; exercise: string; createdAt: string; updatedAt: string
    constructor(id: number, active: boolean, exercise: string, createdAt: string, updatedAt: string) {
        this.id = id;
        this.exercise = exercise;
        this.active = active;
        this.updatedAt = updatedAt;
        this.createdAt = createdAt;
    }
}
