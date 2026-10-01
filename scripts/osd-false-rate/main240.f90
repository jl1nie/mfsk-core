program falserate240
  implicit none
  integer, parameter :: N=240
  integer*1 apmask(N), message101(101), cw(N)
  real llr(N)
  integer i, it, ntype, nharderror, nn, crc, le60, seed, nord
  real dmin, u1, u2
  integer(8) s
  character(len=32) arg
  nn=20000
  seed=1
  nord=2
  if(command_argument_count().ge.1) then
     call get_command_argument(1,arg); read(arg,*) nn
  endif
  if(command_argument_count().ge.2) then
     call get_command_argument(2,arg); read(arg,*) seed
  endif
  if(command_argument_count().ge.3) then
     call get_command_argument(3,arg); read(arg,*) nord
  endif
  s = 88172645463325252_8 + seed*7919_8
  apmask=0
  crc=0; le60=0
  do it=1,nn
     do i=1,N
        call nextu(s,u1); call nextu(s,u2)
        llr(i)=2.83*sqrt(-2.0*log(u1))*cos(6.2831853*u2)
     enddo
     call decode240_101(llr,101,2,nord,apmask,message101,cw,ntype,nharderror,dmin)
     if(nharderror.gt.0) then
        crc=crc+1
        if(nharderror.le.60) le60=le60+1
     endif
     if(mod(it,10).eq.0) then
        write(*,'(a,i6,a,i6,a,i4)') 'PROGRESS it=',it,' of ',nn,' crc_pass=',crc
        call flush(6)
     endif
  enddo
  write(*,'(a,i9,a,i6,a,i6)') 'FALSERATE240 n=',nn,' crc_pass=',crc,' hard<=60=',le60
contains
  subroutine nextu(s,u)
    integer(8), intent(inout) :: s
    real, intent(out) :: u
    s = ieor(s, ishft(s,13)); s = ieor(s, ishft(s,-7)); s = ieor(s, ishft(s,17))
    u = (real(iand(ishft(s,-40),16777215_8))+0.5)/16777216.0
  end subroutine
end program
