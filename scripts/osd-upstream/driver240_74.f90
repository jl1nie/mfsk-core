program decode74_driver
! Feeds WSJT-X's decode240_74_owned (unmodified, v3.3.0-beta1) the records in
! argv(1), writes its outputs to argv(2), and times the calls (repeated
! argv(3) times). One workspace is shared by all records, as the decoder keeps
! one per instance (its generator cache is keyed by Keff only).
  use fst4_osd_workspace, only: fst4_osd_workspace_type
  use fst4_ldpc_74, only: decode240_74_owned
  implicit none
  integer, parameter :: N=240
  integer :: nrec, i, irep, nrep, keff, maxosd, norder, ntype, nharderror
  integer*1, allocatable :: ap(:,:), cwo(:,:), m74o(:,:)
  real, allocatable :: llr(:,:), dmo(:)
  integer, allocatable :: kk(:), mo(:), no(:), nto(:), nho(:)
  integer*1 message74(74), cw(N)
  real dmin
  character(len=256) :: fin, fout, arg
  integer(8) :: t0, t1, rate
  type(fst4_osd_workspace_type) :: work
  call get_command_argument(1, fin); call get_command_argument(2, fout)
  call get_command_argument(3, arg); read(arg,*) nrep
  open(10, file=fin, access='stream', form='unformatted', status='old')
  read(10) nrec
  allocate(ap(N,nrec), llr(N,nrec), kk(nrec), mo(nrec), no(nrec))
  allocate(cwo(N,nrec), m74o(74,nrec), dmo(nrec), nto(nrec), nho(nrec))
  do i=1,nrec
     read(10) kk(i), mo(i), no(i), ap(:,i), llr(:,i)
  enddo
  close(10)
  call system_clock(t0, rate)
  do irep=1,nrep
     do i=1,nrec
        cw=0; message74=0; ntype=0; nharderror=-1; dmin=0.0
        call decode240_74_owned(work,llr(:,i),kk(i),mo(i),no(i),ap(:,i), &
             message74,cw,ntype,nharderror,dmin)
        if(irep.eq.1) then
           cwo(:,i)=cw; m74o(:,i)=message74; nto(i)=ntype; nho(i)=nharderror; dmo(i)=dmin
        endif
     enddo
  enddo
  call system_clock(t1)
  write(*,'(a,i0,a,f10.2,a)') 'records=', nrec, '  us/call=', &
       1.0d6*dble(t1-t0)/dble(rate)/dble(nrec*nrep), ''
  open(11, file=fout, access='stream', form='unformatted', status='replace')
  do i=1,nrec
     write(11) nto(i), nho(i), dmo(i), cwo(:,i), m74o(:,i)
  enddo
  close(11)
end program
