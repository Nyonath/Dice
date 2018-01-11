testClient = CClient();

n = 0;

while(n < 3)
   n = n + 1;
   testClient.readValues();
end

fclose(testClient.mSocket);
delete(testClientf.mSocket);
