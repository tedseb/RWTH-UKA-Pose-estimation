import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';
import { Observable, Subject, timer } from 'rxjs';
import { switchMap, tap, share, retry, takeUntil } from 'rxjs/operators';

@Injectable({
  providedIn: 'root'
})
export class EmergenciesService {

  private emergencies$: Observable<any[]>;

  private stopPolling = new Subject();
  

  constructor(private http: HttpClient) { 
    this.emergencies$ = timer(0, 1000).pipe(
      switchMap(() => http.get<any[]>('/api/emergencies/all')), retry(), share(), takeUntil(this.stopPolling)
    )
  }

  getAllEmergencies(): Observable<any[]> {
    return this.emergencies$;
  };

  ngOnDestroy() {
    this.stopPolling.next();
 }
}
