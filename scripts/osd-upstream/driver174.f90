program osd_driver
! Feeds WSJT-X's osd174_91 (unmodified) the records in argv(1), writes its
! outputs to argv(2), and times the calls (repeated argv(3) times).
  implicit none
  integer, parameter :: N=174
  integer :: nrec, i, irep, nrep, ndeep, nhardmin
  integer*1, allocatable :: ap(:,:), cwo(:,:)
  real, allocatable :: llr(:,:), dmo(:)
  integer, allocatable :: nd(:), nho(:)
  integer*1 message91(91), cw(N)
  real dmin
  character(len=256) :: fin, fout, arg
  integer(8) :: t0, t1, rate
  call get_command_argument(1, fin); call get_command_argument(2, fout)
  call get_command_argument(3, arg); read(arg,*) nrep
  open(10, file=fin, access='stream', form='unformatted', status='old')
  read(10) nrec
  allocate(ap(N,nrec), llr(N,nrec), nd(nrec), cwo(N,nrec), dmo(nrec), nho(nrec))
  do i=1,nrec
     read(10) nd(i), ap(:,i), llr(:,i)
  enddo
  close(10)
  call system_clock(t0, rate)
  do irep=1,nrep
     do i=1,nrec
        call osd174_91(llr(:,i),91,ap(:,i),nd(i),message91,cw,nhardmin,dmin)
        if(irep.eq.1) then
           cwo(:,i)=cw; nho(i)=nhardmin; dmo(i)=dmin
        endif
     enddo
  enddo
  call system_clock(t1)
  write(*,'(a,i0,a,f10.2,a)') 'records=', nrec, '  us/call=', &
       1.0d6*dble(t1-t0)/dble(rate)/dble(nrec*nrep), ''
  open(11, file=fout, access='stream', form='unformatted', status='replace')
  do i=1,nrec
     write(11) nho(i), dmo(i), cwo(:,i)
  enddo
  close(11)
end program
