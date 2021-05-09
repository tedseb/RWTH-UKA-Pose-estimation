import { Injectable } from '@angular/core';
import { BehaviorSubject } from 'rxjs';
import { Spot } from '../shared/models/spot';
import { DataService } from './data.service';
import { WebSocketSubject } from 'rxjs/webSocket';


@Injectable({
  providedIn: 'root'
})
export class SpotsService {

  socket: WebSocketSubject<any>;
  spots: BehaviorSubject<Spot[]>;
  tableCols: string[] = ['id', 'exercise', 'active', 'updatedAt'];

  constructor(private dataService: DataService) { 
    this.socket = dataService.getWebsocket();
    this.socket.subscribe(message => {
      this.spots.next(message);
    });
    this.spots = new BehaviorSubject<Spot[]>([]);
    this.tableCols = this.tableCols;
  }
}
