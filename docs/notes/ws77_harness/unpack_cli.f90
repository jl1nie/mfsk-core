program unpack_cli
  ! args: mycall13 [dxcall13]
  ! stdin lines: "L <c77>" = learn (unpack, discard); "U <c77>" = unpack and print
  use packjt77
  character*77 c77
  character*37 msg
  character*13 mycall
  character*1 cmd
  character*256 arg, line
  integer ios
  logical ok
  mycall=' '
  if (command_argument_count() .ge. 1) then
     call get_command_argument(1, arg); mycall=trim(arg)
  endif
  mycall13=mycall
  do
     read(*,'(a)',iostat=ios) line
     if (ios.ne.0) exit
     cmd=line(1:1); c77=line(3:79)
     msg=' '
     call unpack77(c77, 1, msg, ok)
     if (cmd.eq.'U') write(*,'(l1,1x,a)') ok, trim(msg)
  enddo
end program
