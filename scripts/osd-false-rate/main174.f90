program falserate
  implicit none
  integer, parameter :: N=174
  integer*1 apmask(N), message91(91), cw(N)
  real llr(N)
  integer i, it, ntype, nharderror, nn, crc, le36, le30, seed
  real dmin, u1, u2
  integer(8) s
  character(len=32) arg
  nn=200000
  seed=1
  if(command_argument_count().ge.1) then
     call get_command_argument(1,arg); read(arg,*) nn
  endif
  if(command_argument_count().ge.2) then
     call get_command_argument(2,arg); read(arg,*) seed
  endif
  s = 88172645463325252_8 + seed*7919_8
  apmask=0
  crc=0; le36=0; le30=0
  do it=1,nn
     do i=1,N
        call nextu(s,u1); call nextu(s,u2)
        llr(i)=2.83*sqrt(-2.0*log(u1))*cos(6.2831853*u2)
     enddo
     call decode174_91(llr,91,2,2,apmask,message91,cw,ntype,nharderror,dmin)
     if(nharderror.gt.0) then
        crc=crc+1
        if(nharderror.le.36) le36=le36+1
        if(nharderror.le.30) le30=le30+1
     endif
     if(mod(it,1000).eq.0) then
        write(*,'(a,i8,a,i8,a,i5)') 'PROGRESS it=',it,' of ',nn,' crc_pass=',crc
        call flush(6)
     endif
  enddo
  write(*,'(a,i9,a,i6,a,i6,a,i6)') 'FALSERATE n=',nn,' crc_pass=',crc,' hard<=36=',le36,' hard<=30=',le30
contains
  subroutine nextu(s,u)
    integer(8), intent(inout) :: s
    real, intent(out) :: u
    s = ieor(s, ishft(s,13)); s = ieor(s, ishft(s,-7)); s = ieor(s, ishft(s,17))
    u = (real(iand(ishft(s,-40),16777215_8))+0.5)/16777216.0
  end subroutine
end program
