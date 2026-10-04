program pack_cli
  ! stdin: one message per line. stdout: i3 n3 c77 msgsent
  use packjt77
  character*37 msg, msgsent
  character*77 c77
  character*13 mycall
  integer i3, n3, ios
  logical ok
  character*256 arg
  mycall=' '
  if (command_argument_count() .ge. 1) then
     call get_command_argument(1, arg); mycall=trim(arg)
  endif
  mycall13=mycall
  do
     read(*,'(a)',iostat=ios) msg
     if (ios.ne.0) exit
     i3=-1; n3=-1
     call pack77(msg, i3, n3, c77)
     call unpack77(c77, 0, msgsent, ok)
     write(*,'(i2,1x,i2,1x,a77,1x,l1,1x,a)') i3, n3, c77, ok, trim(msgsent)
  enddo
end program
