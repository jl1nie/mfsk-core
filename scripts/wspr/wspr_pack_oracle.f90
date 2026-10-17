! Oracle for mfsk-core's msg::wspr::pack_message: WSJT-X's own wqencode (lib/wqencode.f90, v3.2.0-rc1)
! followed by wqdecode, for ONE message per process (upstream aborts with a runtime error on an
! unreadable power; a process per message keeps that observable).
!
!   wspr_pack_oracle <file>      first line of <file> is the message, passed as character*22
!
! Output, one tab-separated line:   status  ntype  bits  decoded
!   status = ok      wqencode packed it
!            reject  wqencode returned without writing data0 (no branch of it took the text)
!   ntype  = wqencode's ntype
!   bits   = the 50 information bits, MSB first (data0 as pack50 lays them out)
!   decoded= what wqdecode prints for those bits (a type 3 call is `<...>`: the table is empty)
! A runtime error (list-directed read of the power, a bounds violation) is the process exit status,
! not a line.
program wspr_pack_oracle
  implicit none
  character(len=512) :: infile
  character(len=80) :: line
  character(len=22) :: msg, decoded
  integer*1 :: data0(11)
  integer :: ntype, ntype2, i, b
  character(len=50) :: bits
  call get_command_argument(1, infile)
  open(10, file=trim(infile), status='old')
  read(10, '(a)') line
  msg = line
  data0 = 99
  ntype = -99999
  call wqencode(msg, ntype, data0)
  if (data0(11) == 99) then
     write(*,'(a)') 'reject'//char(9)//'0'//char(9)//repeat('0',50)//char(9)
     stop
  end if
  do i = 0, 49
     b = iand(ishft(iand(int(data0(i/8+1)), 255), -(7 - mod(i, 8))), 1)
     bits(i+1:i+1) = char(48 + b)
  end do
  call wqdecode(data0, decoded, ntype2)
  write(*,'(a,a,i0,a,a,a,a)') 'ok', char(9), ntype, char(9), bits, char(9), trim(decoded)
end program wspr_pack_oracle
