#!/bin/php -q
<?php
error_reporting(E_ALL);
if (isset($_POST['direction']) && isset($_POST['address'])){
  $st = $_POST['direction'];
  $address = $_POST['address'];
}
var_dump($_POST);
#$st = 'left';
#$address='10.0.0.108';
$length = strlen($st);
$socket = socket_create(AF_INET, SOCK_STREAM, 0);
$bind = socket_connect($socket,$address,12000);
while ($bind){
  $send = socket_write($socket,$st,$length);
  if($send === true){
    break;
  }
  socket_close($socket);
  if ($send < $length){
    $st = substr($st,$send);
    $length -= $send;
  }else{
    break;
  }
}
?>
