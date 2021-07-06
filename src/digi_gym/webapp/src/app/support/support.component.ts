import { Component, OnInit } from '@angular/core';
import { take } from 'rxjs/operators';
import { EmergenciesService } from '../services/emergencies.service';

@Component({
  selector: 'app-support',
  templateUrl: './support.component.html',
  styleUrls: ['./support.component.scss']
})
export class SupportComponent implements OnInit {

  requests: any;
  
  constructor(private emergencyService: EmergenciesService) {
    this.emergencyService.getAllEmergencies().pipe(take(1)).subscribe(res => {
      this.requests  = res;
    });
  }

  ngOnInit(): void {
  }





}
